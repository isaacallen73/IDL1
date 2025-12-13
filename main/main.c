// ============================================================================
// FEATURE FLAGS - Uncomment to enable specific hardware modules
// ============================================================================
#define ENABLE_I2C_SENSORS    // Enable I2C multiplexer and BMI160 IMU
#define ENABLE_GPS            // Enable GPS module
#define ENABLE_SD_CARD        // Enable SD card logging
#define ENABLE_BLUETOOTH      // Enable BLE data streaming to Android

// ============================================================================
// Common includes
// ============================================================================
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <errno.h>
#include <esp_err.h>
#include <esp_log.h>
#include <esp_system.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include "driver/gpio.h"

// ============================================================================
// I2C Sensor configuration (disabled - hardware not working on current board)
// TODO: Re-enable when moving to final hardware with proper I2C connections
// ============================================================================
#ifdef ENABLE_I2C_SENSORS
#include "driver/i2c.h"
#include <i2cdev.h>
#include "bmi160.h"
#include "bmi160_reg.h"

// I2C Configuration for Seeed Studio XIAO ESP32-C6
// D4 = GPIO22 (SDA), D5 = GPIO23 (SCL)
#define I2C_MASTER_SCL_IO 23  // D5 on XIAO
#define I2C_MASTER_SDA_IO 22  // D4 on XIAO
#define I2C_MASTER_FREQ_HZ 1000000  // 1MHz - Maximum for BMI160
#define I2C_MASTER_NUM I2C_NUM_0
#define I2C_PORT 0

// PCA9548A Multiplexer Configuration
#define PCA9548A_ADDR 0x70
#define BMI160_MUX_CHANNEL_2 2      // First BMI160 on channel 2
#define BMI160_MUX_CHANNEL_3 3      // Second BMI160 on channel 3
#define BMI160_MUX_CHANNEL_4 4      // Third BMI160 on channel 4
#define PCA9548A_CHANNEL(n) (1 << (n))  // Channel select macro
#define NUM_IMUS 3                  // Total number of IMU sensors

// BMI160 I2C Address (depends on SDO pin connection)
#ifdef CONFIG_EXAMPLE_I2C_ADDRESS_GND
#define BMI160_ADDR BMI160_I2C_ADDRESS_GND
#else
#define BMI160_ADDR BMI160_I2C_ADDRESS_VDD
#endif

// Buffering Configuration for High-Speed Data Logging
#define IMU_BUFFER_SIZE 512         // Number of samples to buffer (512 samples = ~320ms at 1600Hz)
#define SD_BATCH_SIZE 256           // Write to SD every N samples (larger = more efficient for SD)
#define BLE_BATCH_SIZE 20           // Send via BLE every N samples (20 × 25 bytes = 500 bytes, fits perfectly in 1 MTU)
#define IMU_QUEUE_SIZE 256          // FreeRTOS queue depth (increased for FIFO mode buffering)
#define BLE_QUEUE_SIZE 256          // Separate queue for BLE transmission (must handle 2400 samples/sec)

// Performance optimization flags
#define ENABLE_PERF_LOGGING 0       // Set to 1 to enable performance logging (reduces max sample rate)
#define ENABLE_DEBUG_LOGGING 0      // Set to 0 to disable all debug logging in hot paths

// IMU Sample Structure - stores one timestamped sensor reading from a single IMU
// Using raw int16 values to minimize BLE bandwidth and avoid float conversion overhead
// __attribute__((packed)) ensures no padding bytes are added
typedef struct __attribute__((packed)) {
    int64_t timestamp_us;  // Microsecond timestamp
    int16_t accX, accY, accZ;   // Raw accelerometer (convert with: value * 2.0 / 16384.0 for ±2g)
    int16_t gyroX, gyroY, gyroZ; // Raw gyroscope (convert with: value * 125.0 / 262.4 for ±125dps)
} imu_sample_t;

// Combined IMU Sample Structure - stores readings from all 3 IMUs at once
typedef struct __attribute__((packed)) {
    int64_t timestamp_us;  // Microsecond timestamp
    imu_sample_t imu2;     // Data from IMU on channel 2
    imu_sample_t imu3;     // Data from IMU on channel 3
    imu_sample_t imu4;     // Data from IMU on channel 4
} combined_imu_sample_t;

// FIFO Configuration and Constants
#define FIFO_MAX_SIZE 1024                  // BMI160 FIFO buffer size in bytes
#define FIFO_FRAME_SIZE_HEADERLESS 12       // Accel (6) + Gyro (6) bytes
#define FIFO_FRAME_SIZE_HEADER 13           // Header (1) + Accel (6) + Gyro (6) bytes
#define FIFO_POLL_INTERVAL_MS 10            // Poll FIFO every 10ms (~8 samples/IMU per read, prevents overflow with BLE active)
#define FIFO_MAX_FRAMES 78                  // ~1024 / 13 bytes per frame
#define SENSOR_TIME_TICK_US 39.0625         // BMI160 sensor time resolution (microseconds per tick)

// FIFO Frame Header Values (from BMI160 datasheet)
#define FIFO_HEADER_ACCEL_GYRO 0x8C         // Both accelerometer and gyroscope data
#define FIFO_HEADER_GYRO_ONLY  0x88         // Gyroscope only
#define FIFO_HEADER_ACCEL_ONLY 0x84         // Accelerometer only
#define FIFO_HEADER_SENSOR_TIME 0x44        // Timestamp frame
#define FIFO_HEADER_SKIP 0x40               // Skip frame (FIFO was full)
#define FIFO_HEADER_CONFIG 0x48             // Configuration change

// Timestamped IMU Sample - includes both ESP32 and BMI160 timestamps for synchronization
typedef struct __attribute__((packed)) {
    uint8_t imu_id;                // IMU identifier (0, 1, or 2)
    int64_t esp32_time_us;         // ESP32 timestamp for cross-sensor alignment
    uint32_t sensor_time_ticks;    // BMI160 internal time for intra-sensor interpolation
    int16_t accel[3];              // Raw accelerometer data (X, Y, Z)
    int16_t gyro[3];               // Raw gyroscope data (X, Y, Z)
} timestamped_imu_sample_t;

// Global queues for inter-task communication
static QueueHandle_t imu_queue = NULL;     // Sensor task → SD writer task (timestamped_imu_sample_t)
static QueueHandle_t ble_queue = NULL;     // SD writer task → BLE task (combined_imu_sample_t)

// Static buffers for data_writer_task (to avoid stack overflow)
// Allocated in global memory instead of on task stack
static combined_imu_sample_t batch_buffer[SD_BATCH_SIZE];
static combined_imu_sample_t ble_batch_buffer[BLE_BATCH_SIZE];  // BLE uses combined samples directly

// FIFO parsing buffer (allocate once, reuse for all reads)
static uint8_t fifo_buffer[FIFO_MAX_SIZE + 4];  // +4 for timestamp frame

// Forward declarations for FIFO parsing functions
int parse_fifo_frames(uint8_t *fifo_data, uint16_t fifo_len, uint8_t imu_id,
                      int64_t read_timestamp_us, timestamped_imu_sample_t *samples,
                      int max_samples);

#endif

// ============================================================================
// GPS configuration
// ============================================================================
#ifdef ENABLE_GPS
#include "driver/uart.h"

// GPS UART Configuration for XIAO ESP32-C6
// GT-U7 GPS Module connected to D6 (GPIO2) and D7 (GPIO3)
#define GPS_UART_NUM UART_NUM_1
#define GPS_TX_PIN 16   // D7 - TX from ESP32 to GPS RX (for sending commands)
#define GPS_RX_PIN 17   // D6 - RX from GPS TX to ESP32 (NMEA data)
#define GPS_BAUD_RATE 9600      // GT-U7 default baud rate
#define GPS_BUF_SIZE 1024
#endif

// ============================================================================
// SD Card configuration
// ============================================================================
#ifdef ENABLE_SD_CARD
#include <sys/unistd.h>
#include <sys/stat.h>
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"

// SD Card SPI Configuration for XIAO ESP32-C6
// SPI2 (HSPI) pins for SD card interface
#define SD_MOSI_PIN     18  // D10 (GPIO18) - MOSI (Master Out Slave In)
#define SD_MISO_PIN     20  // D9 (GPIO20) - MISO (Master In Slave Out)
#define SD_SCK_PIN      19  // D8 (GPIO19) - SCK (Serial Clock)
#define SD_CS_PIN       21  // D3 (GPIO21) - CS (Chip Select)

#define SD_SPI_HOST     SPI2_HOST
#define SD_MOUNT_POINT  "/sdcard"
#define SD_MAX_FILES    5

static sdmmc_card_t *sd_card = NULL;
static FILE *data_log_file = NULL;  // Single binary log file kept open for all sensor data
static char current_data_log_path[64] = {0};  // Track current log file path for renaming

// Forward declarations for SD card functions
esp_err_t sd_card_init(void);
void sd_card_deinit(void);
esp_err_t sd_write_file(const char *filename, const char *data, bool append);
esp_err_t sd_log_gps(const char *nmea_sentence);
esp_err_t sd_open_data_log(void);  // Open binary log file for fast writes
void sd_close_data_log(void);      // Close and flush data log
esp_err_t sd_rename_data_log_with_gps_time(int year, int month, int day, int hour, int minute, int second);  // Rename with GPS timestamp

#ifdef ENABLE_I2C_SENSORS
esp_err_t sd_log_imu_batch(imu_sample_t *samples, size_t count);  // Fast binary write
#endif
#endif

// ============================================================================
// Bluetooth BLE configuration
// ============================================================================
#ifdef ENABLE_BLUETOOTH
#include "nvs_flash.h"
#include "esp_bt.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"

// BLE Service and Characteristic UUIDs
// Custom 128-bit UUIDs for our datalogger service
#define GATTS_SERVICE_UUID_DATALOGGER   0x00FF
#define GATTS_CHAR_UUID_IMU_DATA        0xFF01  // Combined IMU data (all 3 channels)
#define GATTS_CHAR_UUID_GPS_DATA        0xFF02

// GATT Server Configuration
#define GATTS_NUM_HANDLE                8  // Service + 2 characteristics + 2 CCCDs
#define GATTS_DEMO_CHAR_VAL_LEN_MAX     512
#define PREPARE_BUF_MAX_SIZE            1024
#define DEVICE_NAME                     "ESP32-Datalogger"
#define GATTS_TAG                       "BLE_GATTS"

// BLE Profile IDs
#define PROFILE_NUM                     1
#define PROFILE_APP_IDX                 0
#define APP_ID                          0x55

// MTU size - negotiated with client (23-512 bytes)
#define BLE_MTU_SIZE                    512

// Global BLE state variables
static uint16_t ble_conn_id = 0xFFFF;  // Connection ID (0xFFFF = not connected)
static uint16_t ble_gatts_if = 0xFF;   // GATT server interface
static bool ble_is_connected = false;
static uint16_t ble_mtu = 23;          // Current MTU (starts at minimum)
static bool ble_characteristics_ready = false;  // All characteristics initialized

// Characteristic handles
static uint16_t imu_data_handle = 0;   // Combined IMU data (all 3 channels)
static uint16_t gps_data_handle = 0;

// Forward declarations for BLE functions
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param);
esp_err_t ble_init(void);
esp_err_t ble_send_combined_imu_batch(combined_imu_sample_t *samples, size_t count);
esp_err_t ble_send_timestamped_imu_batch(timestamped_imu_sample_t *samples, size_t count);
esp_err_t ble_send_gps_data(const char *nmea_sentence);

#endif // ENABLE_BLUETOOTH

static const char *TAG = "DATALOGGER";

// ============================================================================
// I2C SENSOR CODE - FIFO-Based Buffered Acquisition
// ============================================================================
#ifdef ENABLE_I2C_SENSORS

TaskHandle_t sensor_task_handle = NULL;
TaskHandle_t sd_writer_task_handle = NULL;
TaskHandle_t ble_sender_task_handle = NULL;

// ============================================================================
// FIFO Parsing Functions
// ============================================================================

/**
 * @brief Parse FIFO frames from BMI160 with header mode and timestamp support
 *
 * @param fifo_data Raw FIFO data buffer
 * @param fifo_len Length of FIFO data in bytes
 * @param imu_id IMU identifier (0, 1, or 2)
 * @param read_timestamp_us ESP32 timestamp when FIFO was read
 * @param samples Output buffer for parsed samples
 * @param max_samples Maximum number of samples to parse
 * @return Number of samples parsed
 */
int parse_fifo_frames(uint8_t *fifo_data, uint16_t fifo_len, uint8_t imu_id,
                      int64_t read_timestamp_us, timestamped_imu_sample_t *samples,
                      int max_samples)
{
    int sample_count = 0;
    int pos = 0;

    // PERFORMANCE: Skip timestamp extraction from FIFO for maximum speed
    // We'll use read_timestamp_us for all samples (post-process interpolation on receiver)
    // Check for timestamp frame at end and skip it to avoid parsing errors
    if (fifo_len >= 4) {
        int ts_pos = fifo_len - 4;
        if (fifo_data[ts_pos] == FIFO_HEADER_SENSOR_TIME) {
            fifo_len -= 4;  // Skip timestamp frame (don't parse it)
        }
    }

    // Single pass: parse data frames (optimized for speed)
    while (pos < fifo_len && sample_count < max_samples) {
        uint8_t header = fifo_data[pos];

        // Parse accel+gyro frame (header 0x8C) - most common case first
        if (header == FIFO_HEADER_ACCEL_GYRO) {
            if (pos + 13 > fifo_len) {
                break;  // Incomplete frame - no logging
            }

            // Extract raw sensor data (12 bytes after header)
            // Gyro X, Y, Z (bytes 1-6), Accel X, Y, Z (bytes 7-12)
            samples[sample_count].gyro[0] = (int16_t)(fifo_data[pos+1] | (fifo_data[pos+2] << 8));
            samples[sample_count].gyro[1] = (int16_t)(fifo_data[pos+3] | (fifo_data[pos+4] << 8));
            samples[sample_count].gyro[2] = (int16_t)(fifo_data[pos+5] | (fifo_data[pos+6] << 8));

            samples[sample_count].accel[0] = (int16_t)(fifo_data[pos+7] | (fifo_data[pos+8] << 8));
            samples[sample_count].accel[1] = (int16_t)(fifo_data[pos+9] | (fifo_data[pos+10] << 8));
            samples[sample_count].accel[2] = (int16_t)(fifo_data[pos+11] | (fifo_data[pos+12] << 8));

            samples[sample_count].imu_id = imu_id;

            // PERFORMANCE: No interpolation - use FIFO read timestamp for all samples
            // Post-processing can interpolate on receiving device: timestamp[i] = read_time - (count - i) * 1.25ms
            // This saves ~30 CPU cycles per sample (timestamp extraction + interpolation eliminated)
            samples[sample_count].sensor_time_ticks = 0;  // Not extracted (saves time)
            samples[sample_count].esp32_time_us = read_timestamp_us;

            sample_count++;
            pos += 13;  // Move to next frame
        } else if (header == FIFO_HEADER_SENSOR_TIME) {
            // Timestamp frame (shouldn't happen since we skip at end, but handle just in case)
            pos += 4;
        } else if (header == FIFO_HEADER_SKIP) {
            // Skip frame (FIFO overflow marker)
            pos += 1;
        } else if (header == FIFO_HEADER_CONFIG) {
            // Config change frame
            pos += 1;
        } else {
            // Unknown header - skip byte
            pos++;
        }
    }

    return sample_count;
}

// High-priority sensor polling task
void bmi160_sensor_task(void *pvParameters)
{
    // Initialize I2C device layer
    ESP_ERROR_CHECK(i2cdev_init());
    ESP_LOGI(TAG, "I2C initialized on SDA=GPIO%d, SCL=GPIO%d", I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO);

    // Initialize PCA9548A multiplexer
    i2c_dev_t mux_dev;
    memset(&mux_dev, 0, sizeof(i2c_dev_t));
    ESP_ERROR_CHECK(i2c_dev_create_mutex(&mux_dev));
    mux_dev.port = I2C_PORT;
    mux_dev.addr = PCA9548A_ADDR;
    mux_dev.cfg.sda_io_num = I2C_MASTER_SDA_IO;
    mux_dev.cfg.scl_io_num = I2C_MASTER_SCL_IO;

    // Initialize all 3 BMI160 sensors
    bmi160_t bmi160_dev_ch2, bmi160_dev_ch3, bmi160_dev_ch4;

    // Configuration for all IMUs - set to 800Hz for stable data
    bmi160_conf_t bmi160_conf = {
        .accRange = BMI160_ACC_RANGE_2G,
        .accOdr = BMI160_ACC_ODR_800HZ,  // 800Hz for stable data
        .accAvg = BMI160_ACC_LP_AVG_2,
        .accMode = BMI160_PMU_ACC_NORMAL,
        .gyrRange = BMI160_GYR_RANGE_125DPS,
        .gyrOdr = BMI160_GYR_ODR_800HZ,  // 800Hz for stable data
        .gyrMode = BMI160_PMU_GYR_NORMAL,
        .accUs = 0u
    };

    // Initialize BMI160 on channel 2
    ESP_LOGI(TAG, "Enabling PCA9548A channel %d for BMI160 #1", BMI160_MUX_CHANNEL_2);
    uint8_t channel_mask = PCA9548A_CHANNEL(BMI160_MUX_CHANNEL_2);
    ESP_ERROR_CHECK(i2c_dev_write(&mux_dev, NULL, 0, &channel_mask, 1));

    memset(&bmi160_dev_ch2.i2c_dev, 0, sizeof(i2c_dev_t));
    ESP_LOGI(TAG, "Initializing BMI160 on channel 2");
    ESP_ERROR_CHECK(bmi160_init(&bmi160_dev_ch2, BMI160_ADDR, I2C_PORT, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(bmi160_self_test(&bmi160_dev_ch2));
    ESP_ERROR_CHECK(bmi160_start(&bmi160_dev_ch2, &bmi160_conf));
    ESP_ERROR_CHECK(bmi160_calibrate(&bmi160_dev_ch2));

    // Initialize BMI160 on channel 3
    ESP_LOGI(TAG, "Enabling PCA9548A channel %d for BMI160 #2", BMI160_MUX_CHANNEL_3);
    channel_mask = PCA9548A_CHANNEL(BMI160_MUX_CHANNEL_3);
    ESP_ERROR_CHECK(i2c_dev_write(&mux_dev, NULL, 0, &channel_mask, 1));

    memset(&bmi160_dev_ch3.i2c_dev, 0, sizeof(i2c_dev_t));
    ESP_LOGI(TAG, "Initializing BMI160 on channel 3");
    ESP_ERROR_CHECK(bmi160_init(&bmi160_dev_ch3, BMI160_ADDR, I2C_PORT, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(bmi160_self_test(&bmi160_dev_ch3));
    ESP_ERROR_CHECK(bmi160_start(&bmi160_dev_ch3, &bmi160_conf));
    ESP_ERROR_CHECK(bmi160_calibrate(&bmi160_dev_ch3));

    // Initialize BMI160 on channel 4
    ESP_LOGI(TAG, "Enabling PCA9548A channel %d for BMI160 #3", BMI160_MUX_CHANNEL_4);
    channel_mask = PCA9548A_CHANNEL(BMI160_MUX_CHANNEL_4);
    ESP_ERROR_CHECK(i2c_dev_write(&mux_dev, NULL, 0, &channel_mask, 1));

    memset(&bmi160_dev_ch4.i2c_dev, 0, sizeof(i2c_dev_t));
    ESP_LOGI(TAG, "Initializing BMI160 on channel 4");
    ESP_ERROR_CHECK(bmi160_init(&bmi160_dev_ch4, BMI160_ADDR, I2C_PORT, I2C_MASTER_SDA_IO, I2C_MASTER_SCL_IO));
    ESP_ERROR_CHECK(bmi160_self_test(&bmi160_dev_ch4));
    ESP_ERROR_CHECK(bmi160_start(&bmi160_dev_ch4, &bmi160_conf));
    ESP_ERROR_CHECK(bmi160_calibrate(&bmi160_dev_ch4));

    ESP_LOGI(TAG, "All 3 BMI160 sensors initialized at 800Hz ODR");

    // Enable FIFO mode on all 3 IMUs (header mode + timestamp)
    ESP_LOGI(TAG, "Enabling FIFO mode with headers and timestamps...");

    channel_mask = PCA9548A_CHANNEL(BMI160_MUX_CHANNEL_2);
    ESP_ERROR_CHECK(i2c_dev_write(&mux_dev, NULL, 0, &channel_mask, 1));
    ESP_ERROR_CHECK(bmi160_enable_fifo(&bmi160_dev_ch2, true, true));  // header=true, time=true

    channel_mask = PCA9548A_CHANNEL(BMI160_MUX_CHANNEL_3);
    ESP_ERROR_CHECK(i2c_dev_write(&mux_dev, NULL, 0, &channel_mask, 1));
    ESP_ERROR_CHECK(bmi160_enable_fifo(&bmi160_dev_ch3, true, true));

    channel_mask = PCA9548A_CHANNEL(BMI160_MUX_CHANNEL_4);
    ESP_ERROR_CHECK(i2c_dev_write(&mux_dev, NULL, 0, &channel_mask, 1));
    ESP_ERROR_CHECK(bmi160_enable_fifo(&bmi160_dev_ch4, true, true));

    ESP_LOGI(TAG, "FIFO mode enabled on all 3 IMUs");

    // Performance measurement variables (only used if ENABLE_PERF_LOGGING is set)
#if ENABLE_PERF_LOGGING
    uint32_t total_samples = 0;
    uint32_t fifo_overflow_count = 0;
    uint32_t error_count = 0;
    int64_t start_time = esp_timer_get_time();
    int64_t last_report_time = start_time;
#endif

    // Temporary buffer for parsing FIFO frames from one IMU at a time
    static timestamped_imu_sample_t temp_samples[FIFO_MAX_FRAMES];

    // Lookup tables for fast IMU selection (avoid ternary operators in hot loop)
    static const uint8_t mux_channels[3] = {BMI160_MUX_CHANNEL_2, BMI160_MUX_CHANNEL_3, BMI160_MUX_CHANNEL_4};

    // Initialize device pointer array (can't use static initializer with addresses)
    bmi160_t* imu_devs[3];
    imu_devs[0] = &bmi160_dev_ch2;
    imu_devs[1] = &bmi160_dev_ch3;
    imu_devs[2] = &bmi160_dev_ch4;

    // Main FIFO polling loop - reads buffered samples periodically
    ESP_LOGI(TAG, "Starting FIFO polling loop (interval: %dms)", FIFO_POLL_INTERVAL_MS);

    while (1) {
        int64_t loop_start = esp_timer_get_time();

        // Read FIFO from all 3 IMUs sequentially and send immediately
        for (int imu_idx = 0; imu_idx < 3; imu_idx++) {
            uint8_t mux_channel = mux_channels[imu_idx];
            bmi160_t *imu_dev = imu_devs[imu_idx];

            // Select IMU via multiplexer
            channel_mask = PCA9548A_CHANNEL(mux_channel);
            if (i2c_dev_write(&mux_dev, NULL, 0, &channel_mask, 1) != ESP_OK) {
#if ENABLE_DEBUG_LOGGING
                error_count++;
#endif
                continue;
            }

            // Read FIFO length
            uint16_t fifo_len = 0;
            if (bmi160_get_fifo_length(imu_dev, &fifo_len) != ESP_OK) {
#if ENABLE_DEBUG_LOGGING
                error_count++;
#endif
                continue;
            }

            // Skip if FIFO is empty (always check to avoid reading empty FIFO)
            if (fifo_len == 0) {
                continue;
            }

#if ENABLE_DEBUG_LOGGING
            // Check for FIFO overflow
            if (fifo_len >= 1000) {
                fifo_overflow_count++;
            }
#endif

            // Read FIFO data + timestamp frame (add 4 bytes for timestamp)
            uint16_t read_len = (fifo_len + 4 > FIFO_MAX_SIZE + 4) ? FIFO_MAX_SIZE : fifo_len + 4;

            if (bmi160_read_fifo_data(imu_dev, fifo_buffer, read_len) != ESP_OK) {
#if ENABLE_DEBUG_LOGGING
                error_count++;
#endif
                continue;
            }

            // Capture timestamp immediately after FIFO read
            int64_t read_timestamp = esp_timer_get_time();

            // Parse FIFO frames
            int num_samples = parse_fifo_frames(fifo_buffer, fifo_len, imu_idx,
                                               read_timestamp, temp_samples, FIFO_MAX_FRAMES);

#if ENABLE_DEBUG_LOGGING
            // If we got very few or no samples from a large FIFO, it's likely corrupted
            if (num_samples == 0 && fifo_len > 50) {
                bmi160_flush_fifo(imu_dev);
                error_count++;
                continue;
            }
#endif

            if (num_samples > 0) {
#if ENABLE_PERF_LOGGING
                total_samples += num_samples;
#endif

                // Send individual timestamped samples to queue immediately
                // xQueueSend with 0 timeout returns immediately if full, so no need to check space
                for (int i = 0; i < num_samples; i++) {
                    xQueueSend(imu_queue, &temp_samples[i], 0);
                }
            }
        }

        // Report performance every second
#if ENABLE_PERF_LOGGING
        int64_t current_time = esp_timer_get_time();
        if (current_time - last_report_time >= 1000000) {
            float elapsed_sec = (current_time - start_time) / 1000000.0;
            float avg_rate = total_samples / elapsed_sec;
            float expected_rate = 800.0 * 3;  // 800 Hz × 3 IMUs = 2400 Hz

            ESP_LOGI(TAG, "═══ FIFO PERFORMANCE ═══");
            ESP_LOGI(TAG, "Total samples: %lu | Errors: %lu | Overflows: %lu",
                     total_samples, error_count, fifo_overflow_count);
            ESP_LOGI(TAG, "Sample rate: %.1f Hz (expected: %.1f Hz) - %.1f%%",
                     avg_rate, expected_rate, (avg_rate / expected_rate) * 100.0);
            ESP_LOGI(TAG, "═══════════════════════");

            // Reset counters for next interval
            total_samples = 0;
            error_count = 0;
            fifo_overflow_count = 0;
            last_report_time = current_time;
            start_time = current_time;
        }
#endif

        // Sleep until next poll interval
        int64_t elapsed = esp_timer_get_time() - loop_start;
        int64_t sleep_time = (FIFO_POLL_INTERVAL_MS * 1000) - elapsed;

        if (sleep_time > 0) {
            vTaskDelay(pdMS_TO_TICKS(sleep_time / 1000));
        }
#if ENABLE_DEBUG_LOGGING
        else {
            ESP_LOGW(TAG, "Loop overrun by %lld µs!", -sleep_time);
        }
#endif
    }

    // Cleanup (this code is never reached in normal operation)
    ESP_ERROR_CHECK(bmi160_free(&bmi160_dev_ch2));
    ESP_ERROR_CHECK(bmi160_free(&bmi160_dev_ch3));
    ESP_ERROR_CHECK(bmi160_free(&bmi160_dev_ch4));
}

// Low-priority data writer task (SD card and/or Bluetooth)
#if defined(ENABLE_SD_CARD) || defined(ENABLE_BLUETOOTH)
void data_writer_task(void *pvParameters)
{
    // Static buffers for batching timestamped samples
    static timestamped_imu_sample_t sd_batch[SD_BATCH_SIZE];
    static timestamped_imu_sample_t ble_batch[BLE_BATCH_SIZE];
    size_t sd_batch_count = 0;
    size_t ble_batch_count = 0;
#if ENABLE_PERF_LOGGING
    uint32_t total_written = 0;
#endif

    ESP_LOGI(TAG, "Data writer task started (SD batch: %d, BLE batch: %d samples)", SD_BATCH_SIZE, BLE_BATCH_SIZE);

    while (1) {
        timestamped_imu_sample_t sample;

        // Wait for individual timestamped samples from queue (with 1 second timeout)
        if (xQueueReceive(imu_queue, &sample, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Add to SD batch buffer
            sd_batch[sd_batch_count++] = sample;

            // Add to BLE batch buffer
#ifdef ENABLE_BLUETOOTH
            ble_batch[ble_batch_count++] = sample;

            // Send BLE batch when full
            if (ble_batch_count >= BLE_BATCH_SIZE) {
                // Send raw timestamped samples over BLE (if queue exists)
                if (ble_queue != NULL) {
                    // Check available queue space before sending
                    UBaseType_t available_space = uxQueueSpacesAvailable(ble_queue);

                    if (available_space >= BLE_BATCH_SIZE) {
                        // Queue has space - send entire batch
                        for (int i = 0; i < BLE_BATCH_SIZE; i++) {
                            xQueueSend(ble_queue, &ble_batch[i], 0);
                        }
                        ble_batch_count = 0;
                    } else {
                        // Queue full - drop oldest batch to make room (no logging for performance)
                        ble_batch_count = 0;
                    }
                } else {
                    // No queue - reset batch counter
                    ble_batch_count = 0;
                }
            }
#endif

            // Write SD batch when full
            if (sd_batch_count >= SD_BATCH_SIZE) {
#ifdef ENABLE_SD_CARD
                if (sd_card != NULL && data_log_file != NULL) {
                    // Write timestamped samples as binary data (no logging for performance)
                    size_t written = fwrite(sd_batch, sizeof(timestamped_imu_sample_t), SD_BATCH_SIZE, data_log_file);
#if ENABLE_PERF_LOGGING
                    if (written == SD_BATCH_SIZE) {
                        total_written += SD_BATCH_SIZE;
                        ESP_LOGI(TAG, "Wrote %d timestamped samples to SD (total: %lu)", SD_BATCH_SIZE, total_written);
                    }
#else
                    (void)written;  // Suppress unused variable warning
#endif
                }
#endif
                sd_batch_count = 0;
            }
        } else {
            // Timeout - write partial batch to SD if any
            if (sd_batch_count > 0) {
#ifdef ENABLE_SD_CARD
                if (sd_card != NULL && data_log_file != NULL) {
                    size_t written = fwrite(sd_batch, sizeof(timestamped_imu_sample_t), sd_batch_count, data_log_file);
#if ENABLE_PERF_LOGGING
                    if (written == sd_batch_count) {
                        total_written += sd_batch_count;
                        ESP_LOGI(TAG, "Wrote partial batch: %d samples (total: %lu)", sd_batch_count, total_written);
                    }
#else
                    (void)written;
#endif
                }
#endif
                sd_batch_count = 0;
            }
        }
    }
}

// ============================================================================
// BLE Sender Task - Dedicated task for BLE transmission (non-blocking)
// ============================================================================
#ifdef ENABLE_BLUETOOTH
void ble_sender_task(void *pvParameters)
{
    ESP_LOGI(TAG, "BLE sender task started");
    static timestamped_imu_sample_t ble_send_batch[BLE_BATCH_SIZE];
    static size_t ble_batch_count = 0;

    while (1) {
        timestamped_imu_sample_t sample;

        // Wait for samples from BLE queue (short timeout to drain quickly)
        if (xQueueReceive(ble_queue, &sample, pdMS_TO_TICKS(10)) == pdTRUE) {
            // Always accumulate samples to keep draining the queue
            ble_send_batch[ble_batch_count++] = sample;

            // Send when batch is full
            if (ble_batch_count >= BLE_BATCH_SIZE) {
                if (ble_is_connected) {
                    // Only send if connected (no logging for performance)
                    ble_send_timestamped_imu_batch(ble_send_batch, BLE_BATCH_SIZE);
                }
                // Always reset batch (drop samples if not connected)
                ble_batch_count = 0;
            }
        }
        // Continue immediately to drain queue as fast as possible
    }
}
#endif // ENABLE_BLUETOOTH
#endif // ENABLE_SD_CARD || ENABLE_BLUETOOTH

#endif // ENABLE_I2C_SENSORS

// ============================================================================
// GPS CODE
// ============================================================================
#ifdef ENABLE_GPS

void gps_task(void *pvParameters)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║         GPS MODULE INITIALIZATION              ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "UART: UART%d", GPS_UART_NUM);
    ESP_LOGI(TAG, "RX Pin: GPIO%d (GPS TX)", GPS_RX_PIN);
    ESP_LOGI(TAG, "TX Pin: GPIO%d (GPS RX - optional)", GPS_TX_PIN);
    ESP_LOGI(TAG, "Baud Rate: %d", GPS_BAUD_RATE);
    ESP_LOGI(TAG, "");

    // Configure UART for GPS
    const uart_config_t uart_config = {
        .baud_rate = GPS_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(GPS_UART_NUM, GPS_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(GPS_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(GPS_UART_NUM, GPS_TX_PIN, GPS_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "GPS UART configured successfully");

    // Configure GPS update rate to 5Hz (200ms interval) for smoother data flow
    // UBX-CFG-RATE: measRate=200ms, navRate=1, timeRef=UTC
    const uint8_t ubx_set_5hz[] = {
        0xB5, 0x62,       // Header
        0x06, 0x08,       // CFG-RATE
        0x06, 0x00,       // Length: 6 bytes
        0xC8, 0x00,       // measRate: 200ms (0x00C8 = 200 decimal)
        0x01, 0x00,       // navRate: 1 (every measurement)
        0x01, 0x00,       // timeRef: 1 = UTC
        0xDE, 0x6A        // Checksum
    };

    uart_write_bytes(GPS_UART_NUM, ubx_set_5hz, sizeof(ubx_set_5hz));
    vTaskDelay(pdMS_TO_TICKS(200));

    // Configure GPS to only output GGA and RMC sentences (disable GLL, GSA, GSV, VTG)
    // UBX protocol: Disable unwanted NMEA sentences
    const uint8_t ubx_disable_gll[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x2A}; // GLL
    const uint8_t ubx_disable_gsa[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x02, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x31}; // GSA
    const uint8_t ubx_disable_gsv[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02, 0x38}; // GSV
    const uint8_t ubx_disable_vtg[] = {0xB5, 0x62, 0x06, 0x01, 0x08, 0x00, 0xF0, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x04, 0x46}; // VTG

    uart_write_bytes(GPS_UART_NUM, ubx_disable_gll, sizeof(ubx_disable_gll));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(GPS_UART_NUM, ubx_disable_gsa, sizeof(ubx_disable_gsa));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(GPS_UART_NUM, ubx_disable_gsv, sizeof(ubx_disable_gsv));
    vTaskDelay(pdMS_TO_TICKS(100));
    uart_write_bytes(GPS_UART_NUM, ubx_disable_vtg, sizeof(ubx_disable_vtg));
    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "GPS configured: 5Hz update rate, only GGA and RMC sentences");
    ESP_LOGI(TAG, "Listening for NMEA sentences...");
    ESP_LOGI(TAG, "NOTE: GPS may take 30-60s for cold start (first fix)");
    ESP_LOGI(TAG, "      Ensure GPS antenna has clear view of sky");
    ESP_LOGI(TAG, "");

    // Buffer for GPS data
    uint8_t* data = (uint8_t*) malloc(GPS_BUF_SIZE);
    static char sentence_buffer[256];  // Buffer for individual NMEA sentence
    static int sentence_pos = 0;
    int no_data_count = 0;

    while (1) {
        // Read data from UART with short timeout to prevent buffering
        // GPS outputs at 5Hz (200ms), so read every 100ms to stay ahead
        int len = uart_read_bytes(GPS_UART_NUM, data, GPS_BUF_SIZE - 1, pdMS_TO_TICKS(100));

        if (len > 0) {
            data[len] = '\0';  // Null terminate

            // Process byte by byte to extract individual NMEA sentences
            for (int i = 0; i < len; i++) {
                char c = data[i];

                // Start of new sentence (begins with $)
                if (c == '$') {
                    sentence_pos = 0;
                    sentence_buffer[sentence_pos++] = c;
                }
                // End of sentence (newline)
                else if (c == '\n' || c == '\r') {
                    if (sentence_pos > 0) {
                        sentence_buffer[sentence_pos] = '\0';  // Null terminate

                        // Only process if sentence starts with $ and has content
                        if (sentence_buffer[0] == '$' && sentence_pos > 5) {
                            // ESP_LOGI(TAG, "GPS: %s", sentence_buffer);

                            // Parse RMC sentence for date/time to rename data log file
#ifdef ENABLE_SD_CARD
                            // Check if this is an RMC sentence with valid fix
                            // Format: $GPRMC,hhmmss.ss,A,lat,N,lon,W,speed,course,DDMMYY,mag,E,mode*checksum
                            if (strncmp(sentence_buffer, "$GPRMC", 6) == 0 || strncmp(sentence_buffer, "$GNRMC", 6) == 0) {
                                static bool gps_time_set = false;
                                if (!gps_time_set) {
                                    // Simple parser - extract time and date fields
                                    char *fields[12];
                                    int field_count = 0;
                                    char *token = strtok(sentence_buffer, ",");
                                    while (token != NULL && field_count < 12) {
                                        fields[field_count++] = token;
                                        token = strtok(NULL, ",");
                                    }

                                    // fields[1] = time (hhmmss.ss), fields[2] = status (A=valid), fields[9] = date (DDMMYY)
                                    if (field_count >= 10 && fields[2][0] == 'A' && strlen(fields[1]) >= 6 && strlen(fields[9]) >= 6) {
                                        // Parse time: hhmmss
                                        int hour = (fields[1][0] - '0') * 10 + (fields[1][1] - '0');
                                        int minute = (fields[1][2] - '0') * 10 + (fields[1][3] - '0');
                                        int second = (fields[1][4] - '0') * 10 + (fields[1][5] - '0');

                                        // Parse date: DDMMYY
                                        int day = (fields[9][0] - '0') * 10 + (fields[9][1] - '0');
                                        int month = (fields[9][2] - '0') * 10 + (fields[9][3] - '0');
                                        int year = 2000 + (fields[9][4] - '0') * 10 + (fields[9][5] - '0');

                                        ESP_LOGI(TAG, "GPS time acquired: %04d-%02d-%02d %02d:%02d:%02d",
                                                 year, month, day, hour, minute, second);

                                        // Rename data log file with GPS timestamp
                                        if (sd_rename_data_log_with_gps_time(year, month, day, hour, minute, second) == ESP_OK) {
                                            gps_time_set = true;  // Only rename once
                                        }
                                    }
                                }
                            }

                            // Log to SD card if enabled
                            if (sd_card != NULL) {
                                esp_err_t ret = sd_log_gps(sentence_buffer);
                                if (ret != ESP_OK) {
                                    ESP_LOGW(TAG, "Failed to write GPS data to SD card");
                                }
                            }
#endif

                            // Send over BLE if enabled and connected
#ifdef ENABLE_BLUETOOTH
                            esp_err_t ble_ret = ble_send_gps_data(sentence_buffer);
                            if (ble_ret != ESP_OK && ble_is_connected) {
                                ESP_LOGW(TAG, "Failed to send GPS data over BLE");
                            }
#endif

                            // Yield to scheduler after processing each sentence
                            // This allows IDLE task to run and reset watchdog timer
                            taskYIELD();
                        }

                        sentence_pos = 0;  // Reset for next sentence
                    }
                }
                // Build sentence character by character
                else if (sentence_pos < sizeof(sentence_buffer) - 1) {
                    sentence_buffer[sentence_pos++] = c;
                }
            }

            no_data_count = 0;  // Reset counter
        } else {
            no_data_count++;
            if (no_data_count % 10 == 0) {  // Every 10 seconds
                ESP_LOGW(TAG, "No GPS data received for %d seconds (check wiring/antenna)", no_data_count);
            }
        }
    }

    free(data);
}

#endif // ENABLE_GPS

// ============================================================================
// SD CARD CODE
// ============================================================================
#ifdef ENABLE_SD_CARD

// Initialize SD card and mount filesystem
esp_err_t sd_card_init(void)
{
    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║         SD CARD INITIALIZATION                 ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════╝");
    ESP_LOGI(TAG, "SPI Host: SPI%d", SD_SPI_HOST);
    ESP_LOGI(TAG, "MISO: GPIO%d (D9)", SD_MISO_PIN);
    ESP_LOGI(TAG, "MOSI: GPIO%d (D10)", SD_MOSI_PIN);
    ESP_LOGI(TAG, "SCK:  GPIO%d (D8)", SD_SCK_PIN);
    ESP_LOGI(TAG, "CS:   GPIO%d (D3)", SD_CS_PIN);
    ESP_LOGI(TAG, "Mount point: %s", SD_MOUNT_POINT);
    ESP_LOGI(TAG, "");

    esp_err_t ret;

    // Options for mounting the filesystem
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,   // Auto-format if mount fails
        .max_files = SD_MAX_FILES,
        .allocation_unit_size = 16 * 1024
    };

    // Initialize SPI bus
    spi_bus_config_t bus_cfg = {
        .mosi_io_num = SD_MOSI_PIN,
        .miso_io_num = SD_MISO_PIN,
        .sclk_io_num = SD_SCK_PIN,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = 4000,
    };

    ret = spi_bus_initialize(SD_SPI_HOST, &bus_cfg, SDSPI_DEFAULT_DMA);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize SPI bus: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "SPI bus initialized");

    // Initialize SD card via SPI with slower speed for initialization
    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.gpio_cs = SD_CS_PIN;
    slot_config.host_id = SD_SPI_HOST;

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_DEFAULT;  // Auto-negotiate best speed (typically 20MHz for SPI)

    ESP_LOGI(TAG, "Attempting to mount SD card...");
    ret = esp_vfs_fat_sdspi_mount(SD_MOUNT_POINT, &host, &slot_config, &mount_config, &sd_card);

    if (ret != ESP_OK) {
        if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
            if (sd_card != NULL) {
                ESP_LOGI(TAG, "Card detected but filesystem mount failed:");
                sdmmc_card_print_info(stdout, sd_card);
                ESP_LOGE(TAG, "Possible issues:");
                ESP_LOGE(TAG, "  - Card not formatted as FAT32");
                ESP_LOGE(TAG, "  - Filesystem corruption");
                ESP_LOGE(TAG, "  - Unsupported card type");
                ESP_LOGE(TAG, "Try formatting card as FAT32 on your computer");
            } else {
                ESP_LOGE(TAG, "Check SD card is inserted properly.");
            }
        } else if (ret == ESP_ERR_TIMEOUT) {
            ESP_LOGE(TAG, "SD card timeout - possible causes:");
            ESP_LOGE(TAG, "  - SD card not inserted or bad contact");
            ESP_LOGE(TAG, "  - Incorrect wiring (check MISO/MOSI/SCK/CS pins)");
            ESP_LOGE(TAG, "  - SD card damaged or incompatible");
            ESP_LOGE(TAG, "  - Power supply issue (SD cards need stable 3.3V)");
        } else {
            ESP_LOGE(TAG, "Failed to initialize SD card: %s (0x%x)", esp_err_to_name(ret), ret);
        }
        // Clean up the SPI bus
        spi_bus_free(SD_SPI_HOST);
        return ret;
    }

    // Card has been initialized, print properties
    ESP_LOGI(TAG, "SD card mounted successfully!");
    sdmmc_card_print_info(stdout, sd_card);

    // Print card capacity
    uint64_t cardSize = ((uint64_t) sd_card->csd.capacity) * sd_card->csd.sector_size / (1024 * 1024);
    ESP_LOGI(TAG, "SD card capacity: %llu MB", cardSize);
    ESP_LOGI(TAG, "");

    return ESP_OK;
}

// Unmount SD card
void sd_card_deinit(void)
{
    if (sd_card != NULL) {
        esp_vfs_fat_sdcard_unmount(SD_MOUNT_POINT, sd_card);
        ESP_LOGI(TAG, "SD card unmounted");
        spi_bus_free(SD_SPI_HOST);
        sd_card = NULL;
    }
}

// Write data to a file on SD card
esp_err_t sd_write_file(const char *filename, const char *data, bool append)
{
    char filepath[64];
    snprintf(filepath, sizeof(filepath), "%s/%s", SD_MOUNT_POINT, filename);

    const char *mode = append ? "a" : "w";
    FILE *f = fopen(filepath, mode);
    if (f == NULL) {
        ESP_LOGE(TAG, "Failed to open file for writing: %s", filepath);
        return ESP_FAIL;
    }

    fprintf(f, "%s", data);
    fclose(f);

    return ESP_OK;
}

// Append a line to the GPS log file
esp_err_t sd_log_gps(const char *nmea_sentence)
{
    return sd_write_file("gps_log.txt", nmea_sentence, true);
}

// Open binary data log file and keep it open for fast writes
esp_err_t sd_open_data_log(void)
{
    char filepath[64];

    // Generate unique filename with timestamp
    // Format: sensor_NNNNNN.bin where NNNNNN is seconds since boot
    // TODO: Use GPS time once available for human-readable timestamps
    int64_t timestamp = esp_timer_get_time() / 1000000;  // Convert to seconds
    snprintf(filepath, sizeof(filepath), "%s/sensor_%lld.bin", SD_MOUNT_POINT, timestamp);

    ESP_LOGI(TAG, "Creating new data log file: %s", filepath);

    // Open in write binary mode (creates new file)
    data_log_file = fopen(filepath, "wb");
    if (data_log_file == NULL) {
        ESP_LOGE(TAG, "Failed to create data log file: %s", filepath);
        ESP_LOGE(TAG, "errno=%d (%s)", errno, strerror(errno));

        // Try listing the directory to verify mount point
        ESP_LOGI(TAG, "Attempting to verify SD card is writable...");
        FILE *test = fopen("/sdcard/test.txt", "w");
        if (test == NULL) {
            ESP_LOGE(TAG, "SD card is not writable! errno=%d (%s)", errno, strerror(errno));
            return ESP_FAIL;
        }
        fclose(test);
        remove("/sdcard/test.txt");
        ESP_LOGI(TAG, "SD card is writable, but binary file creation failed");

        return ESP_FAIL;
    }

    // Disable buffering for immediate writes
    setbuf(data_log_file, NULL);  // Unbuffered for lowest latency

    // Save the filepath for later renaming when GPS time is available
    strncpy(current_data_log_path, filepath, sizeof(current_data_log_path) - 1);
    current_data_log_path[sizeof(current_data_log_path) - 1] = '\0';

    ESP_LOGI(TAG, "Opened binary data log: %s", filepath);
    ESP_LOGI(TAG, "File will be renamed with GPS timestamp when available");
    return ESP_OK;
}

// Close and flush data log
void sd_close_data_log(void)
{
    if (data_log_file != NULL) {
        fflush(data_log_file);  // Ensure all data is written
        fclose(data_log_file);
        data_log_file = NULL;
        ESP_LOGI(TAG, "Closed data log file");
    }
}

// Rename data log file with GPS timestamp (call once GPS time is available)
// Format: sensor_YYYYMMDD_HHMMSS.bin
esp_err_t sd_rename_data_log_with_gps_time(int year, int month, int day, int hour, int minute, int second)
{
    if (current_data_log_path[0] == '\0') {
        ESP_LOGW(TAG, "No data log file to rename");
        return ESP_FAIL;
    }

    // Check if already renamed (filename contains underscore date pattern)
    if (strstr(current_data_log_path, "_202") != NULL) {
        ESP_LOGI(TAG, "Data log already renamed with GPS time");
        return ESP_OK;
    }

    char new_filepath[64];
    snprintf(new_filepath, sizeof(new_filepath), "%s/sensor_%04d%02d%02d_%02d%02d%02d.bin",
             SD_MOUNT_POINT, year, month, day, hour, minute, second);

    ESP_LOGI(TAG, "Renaming data log file:");
    ESP_LOGI(TAG, "  From: %s", current_data_log_path);
    ESP_LOGI(TAG, "  To:   %s", new_filepath);

    // Close file before renaming
    bool was_open = (data_log_file != NULL);
    if (was_open) {
        ESP_LOGI(TAG, "Closing file before rename...");
        fflush(data_log_file);
        fclose(data_log_file);
        data_log_file = NULL;

        // Small delay to ensure file is fully closed
        // Keep this minimal to avoid IMU sample loss (256 sample buffer / 2400 Hz = 107ms max)
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    // Verify source file exists before attempting rename
    struct stat st;
    if (stat(current_data_log_path, &st) != 0) {
        ESP_LOGE(TAG, "Source file doesn't exist: %s", current_data_log_path);
        return ESP_FAIL;
    }
    ESP_LOGI(TAG, "Source file exists (size: %ld bytes)", st.st_size);

    // Rename the file
    ESP_LOGI(TAG, "Attempting rename...");
    int rename_result = rename(current_data_log_path, new_filepath);

    if (rename_result != 0) {
        ESP_LOGE(TAG, "rename() failed with errno=%d (%s)", errno, strerror(errno));
        ESP_LOGW(TAG, "FatFS may not support rename on open files - trying copy+delete method...");

        // Fallback: Copy file contents to new name, then delete old file
        FILE *src = fopen(current_data_log_path, "rb");
        if (src == NULL) {
            ESP_LOGE(TAG, "Failed to open source file for copy: errno=%d", errno);
            if (was_open) {
                data_log_file = fopen(current_data_log_path, "ab");
                if (data_log_file != NULL) setbuf(data_log_file, NULL);
            }
            return ESP_FAIL;
        }

        FILE *dst = fopen(new_filepath, "wb");
        if (dst == NULL) {
            ESP_LOGE(TAG, "Failed to create destination file: errno=%d", errno);
            fclose(src);
            if (was_open) {
                data_log_file = fopen(current_data_log_path, "ab");
                if (data_log_file != NULL) setbuf(data_log_file, NULL);
            }
            return ESP_FAIL;
        }

        // Copy in 512-byte chunks
        uint8_t buffer[512];
        size_t bytes_read;
        ESP_LOGI(TAG, "Copying file data...");
        while ((bytes_read = fread(buffer, 1, sizeof(buffer), src)) > 0) {
            fwrite(buffer, 1, bytes_read, dst);
        }

        fclose(src);
        fclose(dst);

        // Delete original file
        ESP_LOGI(TAG, "Deleting original file...");
        if (remove(current_data_log_path) != 0) {
            ESP_LOGE(TAG, "Failed to delete original file: errno=%d", errno);
            // Don't fail - new file exists, just leave old one too
        }

        ESP_LOGI(TAG, "File copied and renamed successfully using fallback method");
    } else {
        ESP_LOGI(TAG, "rename() succeeded");
    }

    // Update the stored path
    strncpy(current_data_log_path, new_filepath, sizeof(current_data_log_path) - 1);
    current_data_log_path[sizeof(current_data_log_path) - 1] = '\0';

    // Reopen file with new name if it was open
    if (was_open) {
        data_log_file = fopen(new_filepath, "ab");
        if (data_log_file == NULL) {
            ESP_LOGE(TAG, "Failed to reopen renamed file: errno=%d (%s)", errno, strerror(errno));
            return ESP_FAIL;
        }
        setbuf(data_log_file, NULL);
    }

    ESP_LOGI(TAG, "Data log file renamed successfully!");
    return ESP_OK;
}

// Fast binary write of IMU batch - minimal overhead
#ifdef ENABLE_I2C_SENSORS
esp_err_t sd_log_imu_batch(imu_sample_t *samples, size_t count)
{
    if (data_log_file == NULL || samples == NULL || count == 0) {
        return ESP_FAIL;
    }

    // Write entire batch as binary data - fastest method
    size_t written = fwrite(samples, sizeof(imu_sample_t), count, data_log_file);

    if (written != count) {
        ESP_LOGW(TAG, "IMU batch write incomplete: %d/%d samples", written, count);
        return ESP_FAIL;
    }

    // Optionally flush periodically (commented out for max speed)
    // fflush(data_log_file);

    return ESP_OK;
}
#endif

#endif // ENABLE_SD_CARD

// ============================================================================
// BLUETOOTH BLE CODE
// ============================================================================
#ifdef ENABLE_BLUETOOTH

// Extended Advertising (BLE 5.0) configuration
#define EXT_ADV_HANDLE  0
#define NUM_EXT_ADV     1

// Forward declaration for building advertising data
static void build_adv_data(uint8_t *buffer, size_t *length);

// Extended advertising parameters (BLE 5.0)
static esp_ble_gap_ext_adv_params_t ext_adv_params = {
    .type = ESP_BLE_GAP_SET_EXT_ADV_PROP_CONNECTABLE,
    .interval_min = 0x20,  // 20ms (units of 0.625ms)
    .interval_max = 0x40,  // 40ms
    .channel_map = ADV_CHNL_ALL,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    .filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
    .primary_phy = ESP_BLE_GAP_PHY_1M,
    .max_skip = 0,
    .secondary_phy = ESP_BLE_GAP_PHY_1M,
    .sid = 0,
    .scan_req_notif = false,
};

// GATT Profile Structure
struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t imu_char_handle;
    uint16_t gps_char_handle;
    esp_bt_uuid_t imu_char_uuid;
    esp_bt_uuid_t gps_char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};

static struct gatts_profile_inst gl_profile = {
    .gatts_cb = gatts_profile_event_handler,
    .gatts_if = ESP_GATT_IF_NONE,
};

// Build BLE advertising data packet programmatically
static void build_adv_data(uint8_t *buffer, size_t *length)
{
    uint8_t *p = buffer;
    size_t name_len = strlen(DEVICE_NAME);

    // Flags
    *p++ = 0x02;  // Length
    *p++ = 0x01;  // Type: Flags
    *p++ = 0x06;  // LE General Discoverable, BR/EDR not supported

    // Complete Local Name
    *p++ = name_len + 1;  // Length (type byte + name)
    *p++ = 0x09;          // Type: Complete Local Name
    memcpy(p, DEVICE_NAME, name_len);
    p += name_len;

    // Complete 128-bit Service UUID
    *p++ = 0x11;  // Length (1 type byte + 16 UUID bytes)
    *p++ = 0x07;  // Type: Complete list of 128-bit UUIDs
    // Service UUID: 000000FF-0000-1000-8000-00805F9B34FB (little-endian)
    *p++ = 0xfb; *p++ = 0x34; *p++ = 0x9b; *p++ = 0x5f;
    *p++ = 0x80; *p++ = 0x00; *p++ = 0x00; *p++ = 0x80;
    *p++ = 0x00; *p++ = 0x10; *p++ = 0x00; *p++ = 0x00;
    *p++ = 0xFF; *p++ = 0x00; *p++ = 0x00; *p++ = 0x00;

    *length = p - buffer;
}

// GAP Event Handler (BLE 5.0 Extended Advertising)
static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    switch (event) {
        case ESP_GAP_BLE_EXT_ADV_SET_PARAMS_COMPLETE_EVT:
            ESP_LOGI(GATTS_TAG, "Extended advertising params set, status=%d", param->ext_adv_set_params.status);

            // Build and set extended advertising data with device name
            uint8_t adv_data[64];
            size_t adv_data_len;
            build_adv_data(adv_data, &adv_data_len);

            esp_ble_gap_config_ext_adv_data_raw(EXT_ADV_HANDLE, adv_data_len, adv_data);
            break;

        case ESP_GAP_BLE_EXT_ADV_DATA_SET_COMPLETE_EVT:
            ESP_LOGI(GATTS_TAG, "Extended advertising data set, status=%d", param->ext_adv_data_set.status);

            // Skip scan response data - advertising data is sufficient for extended advertising
            // Start advertising immediately after advertising data is set
            esp_ble_gap_ext_adv_start(NUM_EXT_ADV, &(esp_ble_gap_ext_adv_t){.instance = EXT_ADV_HANDLE, .duration = 0, .max_events = 0});
            break;

        case ESP_GAP_BLE_EXT_SCAN_RSP_DATA_SET_COMPLETE_EVT:
            // Not used - we skip scan response data for extended advertising
            ESP_LOGI(GATTS_TAG, "Scan response data set, status=%d", param->scan_rsp_set.status);
            break;

        case ESP_GAP_BLE_EXT_ADV_START_COMPLETE_EVT:
            if (param->ext_adv_start.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Extended advertising start failed, status=%d", param->ext_adv_start.status);
            } else {
                ESP_LOGI(GATTS_TAG, "Extended advertising started successfully");
            }
            break;

        case ESP_GAP_BLE_EXT_ADV_STOP_COMPLETE_EVT:
            if (param->ext_adv_stop.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(GATTS_TAG, "Extended advertising stop failed");
            } else {
                ESP_LOGI(GATTS_TAG, "Extended advertising stopped");
            }
            break;

        case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
            ESP_LOGI(GATTS_TAG, "Connection params updated: status=%d", param->update_conn_params.status);
            break;

        case ESP_GAP_BLE_PASSKEY_REQ_EVT:
            ESP_LOGI(GATTS_TAG, "Passkey request - using default (000000)");
            break;

        case ESP_GAP_BLE_NC_REQ_EVT:
            ESP_LOGI(GATTS_TAG, "Numeric comparison request");
            esp_ble_confirm_reply(param->ble_security.ble_req.bd_addr, true);
            break;

        case ESP_GAP_BLE_SEC_REQ_EVT:
            ESP_LOGI(GATTS_TAG, "Security request");
            esp_ble_gap_security_rsp(param->ble_security.ble_req.bd_addr, true);
            break;

        case ESP_GAP_BLE_AUTH_CMPL_EVT:
            if (param->ble_security.auth_cmpl.success) {
                ESP_LOGI(GATTS_TAG, "Authentication complete - bonding successful");
            } else {
                ESP_LOGW(GATTS_TAG, "Authentication failed, fail_reason=0x%x", param->ble_security.auth_cmpl.fail_reason);
            }
            break;

        default:
            break;
    }
}

// GATT Server Event Handler
static void gatts_profile_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(GATTS_TAG, "GATT server registered, status=%d, app_id=%d", param->reg.status, param->reg.app_id);

            gl_profile.service_id.is_primary = true;
            gl_profile.service_id.id.inst_id = 0x00;
            gl_profile.service_id.id.uuid.len = ESP_UUID_LEN_16;
            gl_profile.service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_DATALOGGER;

            // Configure extended advertising (BLE 5.0)
            esp_ble_gap_ext_adv_set_params(EXT_ADV_HANDLE, &ext_adv_params);

            esp_ble_gatts_create_service(gatts_if, &gl_profile.service_id, GATTS_NUM_HANDLE);
            break;

        case ESP_GATTS_CREATE_EVT:
            ESP_LOGI(GATTS_TAG, "Service created, status=%d, service_handle=%d", param->create.status, param->create.service_handle);
            gl_profile.service_handle = param->create.service_handle;

            // Create IMU data characteristic
            gl_profile.imu_char_uuid.len = ESP_UUID_LEN_16;
            gl_profile.imu_char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_IMU_DATA;

            esp_ble_gatts_start_service(gl_profile.service_handle);

            esp_gatt_char_prop_t imu_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
            esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile.service_handle, &gl_profile.imu_char_uuid,
                                                             ESP_GATT_PERM_READ,
                                                             imu_property,
                                                             NULL, NULL);
            if (add_char_ret) {
                ESP_LOGE(GATTS_TAG, "Add IMU char failed, error code=%x", add_char_ret);
            }
            break;

        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGI(GATTS_TAG, "Characteristic added, status=%d, attr_handle=%d, service_handle=%d",
                     param->add_char.status, param->add_char.attr_handle, param->add_char.service_handle);

            // Store the characteristic handle based on UUID
            if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_IMU_DATA) {
                imu_data_handle = param->add_char.attr_handle;
                ESP_LOGI(GATTS_TAG, "Combined IMU data handle (0xFF01): %d", imu_data_handle);

                // Add CCCD (Client Characteristic Configuration Descriptor) for IMU notifications
                esp_bt_uuid_t cccd_uuid;
                cccd_uuid.len = ESP_UUID_LEN_16;
                cccd_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

                uint8_t cccd_init_value[2] = {0x00, 0x00};
                esp_attr_value_t cccd_val = {
                    .attr_max_len = sizeof(cccd_init_value),
                    .attr_len = sizeof(cccd_init_value),
                    .attr_value = cccd_init_value
                };
                esp_ble_gatts_add_char_descr(gl_profile.service_handle, &cccd_uuid,
                                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                               &cccd_val, NULL);
            } else if (param->add_char.char_uuid.uuid.uuid16 == GATTS_CHAR_UUID_GPS_DATA) {
                gps_data_handle = param->add_char.attr_handle;
                ESP_LOGI(GATTS_TAG, "GPS data handle (0xFF02): %d", gps_data_handle);

                // Add CCCD for GPS notifications
                esp_bt_uuid_t cccd_uuid;
                cccd_uuid.len = ESP_UUID_LEN_16;
                cccd_uuid.uuid.uuid16 = ESP_GATT_UUID_CHAR_CLIENT_CONFIG;

                uint8_t cccd_init_value[2] = {0x00, 0x00};
                esp_attr_value_t cccd_val = {
                    .attr_max_len = sizeof(cccd_init_value),
                    .attr_len = sizeof(cccd_init_value),
                    .attr_value = cccd_init_value
                };
                esp_ble_gatts_add_char_descr(gl_profile.service_handle, &cccd_uuid,
                                               ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                               &cccd_val, NULL);
            }
            break;

        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            ESP_LOGI(GATTS_TAG, "Descriptor added, status=%d", param->add_char_descr.status);

            // After IMU descriptor is added, add GPS characteristic
            if (gps_data_handle == 0) {
                gl_profile.gps_char_uuid.len = ESP_UUID_LEN_16;
                gl_profile.gps_char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_GPS_DATA;

                esp_gatt_char_prop_t gps_property = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_NOTIFY;
                esp_err_t add_gps_ret = esp_ble_gatts_add_char(gl_profile.service_handle, &gl_profile.gps_char_uuid,
                                                                 ESP_GATT_PERM_READ,
                                                                 gps_property,
                                                                 NULL, NULL);
                if (add_gps_ret) {
                    ESP_LOGE(GATTS_TAG, "Add GPS char failed, error code=%x", add_gps_ret);
                }
            }
            // After GPS descriptor is added, all characteristics are ready
            else {
                ble_characteristics_ready = true;
                ESP_LOGI(GATTS_TAG, "═══════════════════════════════════════");
                ESP_LOGI(GATTS_TAG, "All BLE characteristics initialized:");
                ESP_LOGI(GATTS_TAG, "  Combined IMU (0xFF01) handle: %d", imu_data_handle);
                ESP_LOGI(GATTS_TAG, "  GPS (0xFF02) handle: %d", gps_data_handle);
                ESP_LOGI(GATTS_TAG, "═══════════════════════════════════════");
            }
            break;

        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "Client connected, conn_id=%d, remote " ESP_BD_ADDR_STR,
                     param->connect.conn_id,
                     ESP_BD_ADDR_HEX(param->connect.remote_bda));

            ble_conn_id = param->connect.conn_id;
            ble_gatts_if = gatts_if;
            ble_is_connected = true;

            // Request MTU exchange
            esp_ble_gatt_set_local_mtu(BLE_MTU_SIZE);

            // Switch to 2M PHY for 2x throughput (BLE 5.0 feature)
            // Note: Requires BLE 5.0+ support on both sides, gracefully falls back to 1M if not supported
            esp_err_t phy_ret = esp_ble_gap_set_preferred_phy(param->connect.remote_bda,
                                                               ESP_BLE_GAP_NO_PREFER_TRANSMIT_PHY | ESP_BLE_GAP_NO_PREFER_RECEIVE_PHY,
                                                               ESP_BLE_GAP_PHY_2M_PREF_MASK,
                                                               ESP_BLE_GAP_PHY_2M_PREF_MASK,
                                                               ESP_BLE_GAP_PHY_OPTIONS_NO_PREF);
            if (phy_ret == ESP_OK) {
                ESP_LOGI(GATTS_TAG, "Requested 2M PHY for higher throughput");
            } else {
                ESP_LOGW(GATTS_TAG, "Failed to request 2M PHY: %s (will use 1M)", esp_err_to_name(phy_ret));
            }

            // Update connection parameters for maximum throughput
            // Using 15ms interval instead of 7.5ms to reduce BLE stack overhead
            // This allows L2CAP buffer to drain between sends
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;       // No latency - immediate response
            conn_params.max_int = 0x0C;    // 15ms max (0x0C * 1.25ms = 15ms)
            conn_params.min_int = 0x0C;    // 15ms min (lock interval)
            conn_params.timeout = 400;     // 4s
            esp_ble_gap_update_conn_params(&conn_params);
            break;

        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGI(GATTS_TAG, "Client disconnected, reason=0x%x", param->disconnect.reason);
            ble_is_connected = false;
            ble_conn_id = 0xFFFF;
            ble_mtu = 23;  // Reset to minimum

            // Restart extended advertising
            esp_ble_gap_ext_adv_start(NUM_EXT_ADV, &(esp_ble_gap_ext_adv_t){.instance = EXT_ADV_HANDLE, .duration = 0, .max_events = 0});
            break;

        case ESP_GATTS_MTU_EVT:
            ESP_LOGI(GATTS_TAG, "MTU exchange complete, MTU=%d", param->mtu.mtu);
            ble_mtu = param->mtu.mtu;
            break;

        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(GATTS_TAG, "Write event: handle=%d, len=%d, is_prep=%d",
                     param->write.handle, param->write.len, param->write.is_prep);

            // Handle CCCD writes (client enabling/disabling notifications)
            if (param->write.len == 2) {
                uint16_t descr_value = param->write.value[1] << 8 | param->write.value[0];

                // Determine which characteristic this belongs to
                const char* char_name = "UNKNOWN";
                if (param->write.handle == imu_data_handle + 1) char_name = "Combined IMU (0xFF01)";
                else if (param->write.handle == gps_data_handle + 1) char_name = "GPS (0xFF02)";

                if (descr_value == 0x0001) {
                    ESP_LOGI(GATTS_TAG, "Notifications ENABLED for %s (handle %d)", char_name, param->write.handle);
                } else if (descr_value == 0x0002) {
                    ESP_LOGI(GATTS_TAG, "Indications ENABLED for %s (handle %d)", char_name, param->write.handle);
                } else if (descr_value == 0x0000) {
                    ESP_LOGI(GATTS_TAG, "Notifications/Indications DISABLED for %s (handle %d)", char_name, param->write.handle);
                }
            }

            // Send write response if needed
            if (param->write.need_rsp) {
                ESP_LOGI(GATTS_TAG, "Sending write response");
                esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id,
                                             ESP_GATT_OK, NULL);
            }
            break;

        case ESP_GATTS_CONF_EVT:
            // Confirmation received for indication
            break;

        default:
            break;
    }
}

// Main GATT Server Callback - routes events to profile handlers
static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile.gatts_if = gatts_if;
        } else {
            ESP_LOGE(GATTS_TAG, "Register app failed, app_id=%04x, status=%d",
                     param->reg.app_id, param->reg.status);
            return;
        }
    }

    if (gatts_if == ESP_GATT_IF_NONE || gatts_if == gl_profile.gatts_if) {
        if (gl_profile.gatts_cb) {
            gl_profile.gatts_cb(event, gatts_if, param);
        }
    }
}

// Send combined IMU batch data over BLE (fragments into MTU-sized packets)
// Sends all 3 IMU channels in one characteristic for maximum efficiency
esp_err_t ble_send_combined_imu_batch(combined_imu_sample_t *samples, size_t count)
{
    if (!ble_is_connected || !ble_characteristics_ready || samples == NULL || count == 0) {
        return ESP_FAIL;
    }

    // Calculate total data size
    size_t total_size = count * sizeof(combined_imu_sample_t);
    uint8_t *data_ptr = (uint8_t *)samples;

    // Lightweight logging - only log occasionally to avoid watchdog timeout
    static uint32_t log_counter = 0;
    if (log_counter++ % 100 == 0) {
        ESP_LOGI(GATTS_TAG, "BLE Combined IMU: %zu samples * %zu bytes = %zu total",
                 count, sizeof(combined_imu_sample_t), total_size);
    }

    // MTU overhead: 3 bytes for ATT header
    size_t usable_mtu = ble_mtu - 3;

    // Fragment and send data
    size_t sent = 0;
    while (sent < total_size) {
        size_t chunk_size = (total_size - sent) > usable_mtu ? usable_mtu : (total_size - sent);

        // Use indicate with need_confirm=false for notification behavior (no ACK required)
        esp_err_t ret = esp_ble_gatts_send_indicate(ble_gatts_if, ble_conn_id, imu_data_handle,
                                                      chunk_size, data_ptr + sent, false);
        if (ret != ESP_OK) {
            ESP_LOGW(GATTS_TAG, "Failed to send combined IMU data chunk, error=0x%x", ret);
            return ret;
        }

        sent += chunk_size;

        // Note: vTaskDelay removed - BLE stack can handle rapid sends
        // With separate BLE task, blocking here doesn't affect sensor sampling
    }

    return ESP_OK;
}

// Send timestamped IMU samples over BLE (individual samples with IMU ID)
esp_err_t ble_send_timestamped_imu_batch(timestamped_imu_sample_t *samples, size_t count)
{
    if (!ble_is_connected || !ble_characteristics_ready || samples == NULL || count == 0) {
        return ESP_FAIL;
    }

    // Calculate total data size
    size_t total_size = count * sizeof(timestamped_imu_sample_t);
    uint8_t *data_ptr = (uint8_t *)samples;

    // MTU overhead: 3 bytes for ATT header
    size_t usable_mtu = ble_mtu - 3;

    // Fragment and send data
    size_t sent = 0;
    while (sent < total_size) {
        size_t chunk_size = (total_size - sent) > usable_mtu ? usable_mtu : (total_size - sent);

        // Use indicate with need_confirm=false for notification behavior (no ACK required)
        esp_err_t ret = esp_ble_gatts_send_indicate(ble_gatts_if, ble_conn_id, imu_data_handle,
                                                      chunk_size, data_ptr + sent, false);
        if (ret != ESP_OK) {
            // BLE congestion or error - drop this batch to prevent blocking
            // Only log occasionally to avoid flooding console
            static uint32_t drop_count = 0;
            if (++drop_count % 100 == 1) {
                ESP_LOGW(GATTS_TAG, "BLE congested, dropped %lu batches", drop_count);
            }
            return ESP_FAIL;  // Return failure but don't log every time
        }

        sent += chunk_size;
    }

    return ESP_OK;
}

// Send GPS NMEA data over BLE
esp_err_t ble_send_gps_data(const char *nmea_sentence)
{
    if (!ble_is_connected || nmea_sentence == NULL) {
        return ESP_FAIL;
    }

    size_t len = strlen(nmea_sentence);
    if (len == 0) {
        return ESP_FAIL;
    }

    // MTU overhead: 3 bytes for ATT header
    size_t usable_mtu = ble_mtu - 3;

    // Send GPS data (fragment if needed, but NMEA sentences are usually <100 bytes)
    size_t sent = 0;
    while (sent < len) {
        size_t chunk_size = (len - sent) > usable_mtu ? usable_mtu : (len - sent);

        // Use indicate with need_confirm=false for notification behavior (no ACK required)
        esp_err_t ret = esp_ble_gatts_send_indicate(ble_gatts_if, ble_conn_id, gps_data_handle,
                                                      chunk_size, (uint8_t *)(nmea_sentence + sent), false);
        if (ret != ESP_OK) {
            ESP_LOGW(GATTS_TAG, "Failed to send GPS data, error=0x%x", ret);
            return ret;
        }

        sent += chunk_size;
    }

    return ESP_OK;
}

// Initialize BLE stack and GATT server
esp_err_t ble_init(void)
{
    ESP_LOGI(GATTS_TAG, "");
    ESP_LOGI(GATTS_TAG, "╔════════════════════════════════════════════════╗");
    ESP_LOGI(GATTS_TAG, "║         BLUETOOTH BLE INITIALIZATION           ║");
    ESP_LOGI(GATTS_TAG, "╚════════════════════════════════════════════════╝");
    ESP_LOGI(GATTS_TAG, "Device Name: %s", DEVICE_NAME);
    ESP_LOGI(GATTS_TAG, "Service UUID: 0x%04X", GATTS_SERVICE_UUID_DATALOGGER);
    ESP_LOGI(GATTS_TAG, "Combined IMU Characteristic UUID: 0x%04X (all 3 channels)", GATTS_CHAR_UUID_IMU_DATA);
    ESP_LOGI(GATTS_TAG, "GPS Characteristic UUID: 0x%04X", GATTS_CHAR_UUID_GPS_DATA);
    ESP_LOGI(GATTS_TAG, "");

    esp_err_t ret;

    // Initialize NVS (required for BLE)
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(GATTS_TAG, "NVS initialized");

    // Release classic Bluetooth memory (we only need BLE)
    ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));

    // Initialize BT controller
    esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
    ret = esp_bt_controller_init(&bt_cfg);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Initialize BT controller failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Enable BLE mode
    ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Enable BT controller failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(GATTS_TAG, "BT controller enabled");

    // Initialize Bluedroid stack
    ret = esp_bluedroid_init();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Initialize Bluedroid failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_bluedroid_enable();
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Enable Bluedroid failed: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(GATTS_TAG, "Bluedroid stack enabled");

    // Set security parameters - disable bonding/authentication for simplicity
    esp_ble_auth_req_t auth_req = ESP_LE_AUTH_NO_BOND;  // No bonding required
    esp_ble_io_cap_t iocap = ESP_IO_CAP_NONE;           // No input/output capability
    uint8_t key_size = 16;
    uint8_t init_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t rsp_key = ESP_BLE_ENC_KEY_MASK | ESP_BLE_ID_KEY_MASK;
    uint8_t auth_option = ESP_BLE_ONLY_ACCEPT_SPECIFIED_AUTH_DISABLE;

    esp_ble_gap_set_security_param(ESP_BLE_SM_AUTHEN_REQ_MODE, &auth_req, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_IOCAP_MODE, &iocap, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_MAX_KEY_SIZE, &key_size, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_ONLY_ACCEPT_SPECIFIED_SEC_AUTH, &auth_option, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_INIT_KEY, &init_key, sizeof(uint8_t));
    esp_ble_gap_set_security_param(ESP_BLE_SM_SET_RSP_KEY, &rsp_key, sizeof(uint8_t));

    ESP_LOGI(GATTS_TAG, "Security configured: No bonding required");

    // Register callbacks
    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "GATTS register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "GAP register callback failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register application profile
    ret = esp_ble_gatts_app_register(APP_ID);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "GATTS app register failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Set local MTU
    ret = esp_ble_gatt_set_local_mtu(BLE_MTU_SIZE);
    if (ret) {
        ESP_LOGE(GATTS_TAG, "Set local MTU failed: %s", esp_err_to_name(ret));
    }

    ESP_LOGI(GATTS_TAG, "BLE initialization complete");
    ESP_LOGI(GATTS_TAG, "");

    return ESP_OK;
}

#endif // ENABLE_BLUETOOTH

// ============================================================================
// MAIN APPLICATION
// ============================================================================
void app_main(void)
{
    // Reduce SPI driver log verbosity to avoid spam from SD card operations
    esp_log_level_set("spi_master", ESP_LOG_INFO);

    ESP_LOGI(TAG, "");
    ESP_LOGI(TAG, "╔════════════════════════════════════════════════╗");
    ESP_LOGI(TAG, "║     DATALOGGER V2 - XIAO ESP32-C6              ║");
    ESP_LOGI(TAG, "╚════════════════════════════════════════════════╝");
#ifdef ENABLE_I2C_SENSORS
    ESP_LOGI(TAG, "DEBUG: sizeof(imu_sample_t) = %zu bytes", sizeof(imu_sample_t));
    ESP_LOGI(TAG, "DEBUG: sizeof(combined_imu_sample_t) = %zu bytes (3 IMUs)", sizeof(combined_imu_sample_t));
    ESP_LOGI(TAG, "DEBUG: sizeof(int64_t) = %zu, sizeof(float) = %zu", sizeof(int64_t), sizeof(float));
#endif
    ESP_LOGI(TAG, "Enabled modules:");
#ifdef ENABLE_I2C_SENSORS
    ESP_LOGI(TAG, "  ✓ I2C Sensors (3x BMI160 IMUs @ 800Hz via PCA9548A mux)");
#else
    ESP_LOGI(TAG, "  ✗ I2C Sensors (disabled)");
#endif
#ifdef ENABLE_GPS
    ESP_LOGI(TAG, "  ✓ GPS Module");
#else
    ESP_LOGI(TAG, "  ✗ GPS Module (disabled)");
#endif
#ifdef ENABLE_SD_CARD
    ESP_LOGI(TAG, "  ✓ SD Card Logging");
#else
    ESP_LOGI(TAG, "  ✗ SD Card Logging (disabled)");
#endif
#ifdef ENABLE_BLUETOOTH
    ESP_LOGI(TAG, "  ✓ Bluetooth BLE (Data Streaming)");
#else
    ESP_LOGI(TAG, "  ✗ Bluetooth BLE (disabled)");
#endif
    ESP_LOGI(TAG, "");

    // Initialize Bluetooth BLE (if enabled)
#ifdef ENABLE_BLUETOOTH
    esp_err_t ble_ret = ble_init();
    if (ble_ret != ESP_OK) {
        ESP_LOGW(TAG, "BLE initialization failed - continuing without Bluetooth");
    }
#endif

    // Initialize SD card (if enabled)
#ifdef ENABLE_SD_CARD
    esp_err_t sd_ret = sd_card_init();
    if (sd_ret != ESP_OK) {
        ESP_LOGW(TAG, "SD card initialization failed - continuing without SD logging");
    } else {
        // Open binary data log file and keep it open for fast writes
        sd_ret = sd_open_data_log();
        if (sd_ret != ESP_OK) {
            ESP_LOGW(TAG, "Failed to open data log file");
        }
    }
#endif

#ifdef ENABLE_I2C_SENSORS
    // Create queue for IMU samples
    imu_queue = xQueueCreate(IMU_QUEUE_SIZE, sizeof(timestamped_imu_sample_t));
    if (imu_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create IMU queue");
    } else {
        ESP_LOGI(TAG, "Created IMU queue: %d slots", IMU_QUEUE_SIZE);

        // Create BLE queue (separate from SD queue for decoupling)
        #ifdef ENABLE_BLUETOOTH
        ble_queue = xQueueCreate(BLE_QUEUE_SIZE, sizeof(timestamped_imu_sample_t));
        if (ble_queue == NULL) {
            ESP_LOGE(TAG, "Failed to create BLE queue");
        } else {
            ESP_LOGI(TAG, "Created BLE queue: %d slots", BLE_QUEUE_SIZE);

            // Create BLE sender task (priority 2 - lower than SD writer)
            xTaskCreate(ble_sender_task, "ble_sender", configMINIMAL_STACK_SIZE * 6, NULL, 2, &ble_sender_task_handle);
        }
        #endif

        // Create high-priority sensor polling task (priority 10)
        xTaskCreate(bmi160_sensor_task, "sensor_poll", configMINIMAL_STACK_SIZE * 8, NULL, 10, &sensor_task_handle);

        // Create low-priority data writer task (priority 3) for SD and/or BLE
        // This task drains the IMU queue, so it's needed even if only BLE is enabled
        #if defined(ENABLE_SD_CARD) || defined(ENABLE_BLUETOOTH)
        xTaskCreate(data_writer_task, "data_writer", configMINIMAL_STACK_SIZE * 10, NULL, 3, &sd_writer_task_handle);
        #endif
    }
#endif

#ifdef ENABLE_GPS
    xTaskCreate(gps_task, "gps_task", configMINIMAL_STACK_SIZE * 4, NULL, 5, NULL);
#endif

#if !defined(ENABLE_I2C_SENSORS) && !defined(ENABLE_GPS) && !defined(ENABLE_SD_CARD)
    ESP_LOGW(TAG, "No modules enabled! Enable at least one module at the top of main.c");
#endif
}
