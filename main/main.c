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
#define BLE_BATCH_SIZE 20           // Send via BLE every N samples (fits in 1 MTU: 20 × 25 bytes = 500 bytes < 509 usable MTU)
#define IMU_QUEUE_SIZE 128          // FreeRTOS queue depth (increased for FIFO mode buffering)
#define BLE_QUEUE_SIZE 128          // Separate queue for BLE transmission (must handle 2400 samples/sec)

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
#define FIFO_POLL_INTERVAL_MS 15            // Poll FIFO every 15ms (~12 samples/IMU per read, prevents overflow with BLE active)
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

// Forward declarations for SD card functions
esp_err_t sd_card_init(void);
void sd_card_deinit(void);
esp_err_t sd_write_file(const char *filename, const char *data, bool append);
esp_err_t sd_log_gps(const char *nmea_sentence);
esp_err_t sd_open_data_log(void);  // Open binary log file for fast writes
void sd_close_data_log(void);      // Close and flush data log

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
    uint32_t last_sensor_time = 0;
    bool found_timestamp = false;
    int pos = 0;

    // First pass: find timestamp frame at the end (if present)
    // Read backwards through buffer looking for timestamp header 0x44
    for (int i = fifo_len - 4; i >= 0; i--) {
        if (fifo_data[i] == FIFO_HEADER_SENSOR_TIME && i + 3 < fifo_len) {
            // Extract 24-bit sensor time (LSB first)
            last_sensor_time = fifo_data[i+1] |
                             ((uint32_t)fifo_data[i+2] << 8) |
                             ((uint32_t)fifo_data[i+3] << 16);
            found_timestamp = true;
            ESP_LOGD(TAG, "IMU %d: Found timestamp frame at offset %d, sensor_time=%lu",
                     imu_id, i, last_sensor_time);
            break;
        }
    }

    // Second pass: parse data frames
    while (pos < fifo_len && sample_count < max_samples) {
        uint8_t header = fifo_data[pos];

        // Check for timestamp frame (skip it, already extracted)
        if (header == FIFO_HEADER_SENSOR_TIME) {
            pos += 4;  // Header + 3 bytes of timestamp
            continue;
        }

        // Check for skip frame (FIFO overflow)
        if (header == FIFO_HEADER_SKIP) {
            ESP_LOGW(TAG, "IMU %d: FIFO skip frame detected (overflow!)", imu_id);
            pos += 1;
            continue;
        }

        // Check for config change frame
        if (header == FIFO_HEADER_CONFIG) {
            ESP_LOGD(TAG, "IMU %d: Config change frame", imu_id);
            pos += 1;
            continue;
        }

        // Parse accel+gyro frame (header 0x8C)
        if (header == FIFO_HEADER_ACCEL_GYRO) {
            if (pos + 13 > fifo_len) {
                ESP_LOGW(TAG, "IMU %d: Incomplete frame at end of FIFO", imu_id);
                break;  // Incomplete frame
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

            // Interpolate timestamp if we have sensor_time
            if (found_timestamp) {
                // Calculate how many samples back from the last one (which has last_sensor_time)
                int samples_from_end = (fifo_len - pos - 13) / 13;  // Remaining frames after this one

                // Interpolate sensor time backwards (800Hz = 32 ticks per sample)
                uint32_t sample_sensor_time = last_sensor_time - (samples_from_end * 32);
                samples[sample_count].sensor_time_ticks = sample_sensor_time;

                // Interpolate ESP32 timestamp backwards (800Hz = 1250µs per sample)
                int64_t time_offset_us = samples_from_end * 1250;
                samples[sample_count].esp32_time_us = read_timestamp_us - time_offset_us;
            } else {
                // No timestamp available, just use read time
                samples[sample_count].sensor_time_ticks = 0;
                samples[sample_count].esp32_time_us = read_timestamp_us;
            }

            sample_count++;
            pos += 13;  // Move to next frame
        } else {
            // Unknown header - skip byte and continue
            ESP_LOGW(TAG, "IMU %d: Unknown FIFO header 0x%02X at pos %d", imu_id, header, pos);
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

    // Performance measurement variables
    uint32_t total_samples = 0;
    uint32_t fifo_overflow_count = 0;
    uint32_t error_count = 0;
    int64_t start_time = esp_timer_get_time();
    int64_t last_report_time = start_time;

    // Temporary buffer for parsing FIFO frames from one IMU at a time
    static timestamped_imu_sample_t temp_samples[FIFO_MAX_FRAMES];

    // Main FIFO polling loop - reads buffered samples periodically
    ESP_LOGI(TAG, "Starting FIFO polling loop (interval: %dms)", FIFO_POLL_INTERVAL_MS);

    while (1) {
        int64_t loop_start = esp_timer_get_time();

        // Read FIFO from all 3 IMUs sequentially and send immediately
        for (int imu_idx = 0; imu_idx < 3; imu_idx++) {
            uint8_t mux_channel = (imu_idx == 0) ? BMI160_MUX_CHANNEL_2 :
                                 (imu_idx == 1) ? BMI160_MUX_CHANNEL_3 :
                                                  BMI160_MUX_CHANNEL_4;

            bmi160_t *imu_dev = (imu_idx == 0) ? &bmi160_dev_ch2 :
                                (imu_idx == 1) ? &bmi160_dev_ch3 :
                                                 &bmi160_dev_ch4;

            // Select IMU via multiplexer
            channel_mask = PCA9548A_CHANNEL(mux_channel);
            if (i2c_dev_write(&mux_dev, NULL, 0, &channel_mask, 1) != ESP_OK) {
                ESP_LOGW(TAG, "IMU %d: Failed to select multiplexer channel", imu_idx);
                error_count++;
                continue;
            }

            // Read FIFO length
            uint16_t fifo_len = 0;
            if (bmi160_get_fifo_length(imu_dev, &fifo_len) != ESP_OK) {
                ESP_LOGW(TAG, "IMU %d: Failed to read FIFO length", imu_idx);
                error_count++;
                continue;
            }

            // Skip if FIFO is empty
            if (fifo_len == 0) {
                continue;
            }

            // Check for FIFO overflow
            if (fifo_len >= 1000) {
                ESP_LOGW(TAG, "IMU %d: FIFO near full (%d bytes) - possible overflow!", imu_idx, fifo_len);
                fifo_overflow_count++;
            }

            // Read FIFO data + timestamp frame (add 4 bytes for timestamp)
            uint16_t read_len = (fifo_len + 4 > FIFO_MAX_SIZE + 4) ? FIFO_MAX_SIZE : fifo_len + 4;

            if (bmi160_read_fifo_data(imu_dev, fifo_buffer, read_len) != ESP_OK) {
                ESP_LOGW(TAG, "IMU %d: Failed to read FIFO data", imu_idx);
                error_count++;
                continue;
            }

            // Capture timestamp immediately after FIFO read
            int64_t read_timestamp = esp_timer_get_time();

            // Parse FIFO frames
            int num_samples = parse_fifo_frames(fifo_buffer, fifo_len, imu_idx,
                                               read_timestamp, temp_samples, FIFO_MAX_FRAMES);

            // If we got very few or no samples from a large FIFO, it's likely corrupted
            // This can happen when BLE causes timing issues or FIFO overflows
            if (num_samples == 0 && fifo_len > 50) {
                ESP_LOGW(TAG, "IMU %d: Failed to parse %d bytes of FIFO (corrupted data), flushing FIFO...",
                         imu_idx, fifo_len);
                bmi160_flush_fifo(imu_dev);
                error_count++;
                continue;
            }

            if (num_samples > 0) {
                ESP_LOGD(TAG, "IMU %d: Parsed %d samples from %d bytes of FIFO",
                         imu_idx, num_samples, fifo_len);
                total_samples += num_samples;

                // Send individual timestamped samples to queue immediately
                // Don't wait to combine - let data_writer_task handle alignment
                if (imu_queue != NULL) {
                    for (int i = 0; i < num_samples; i++) {
                        if (xQueueSend(imu_queue, &temp_samples[i], 0) != pdTRUE) {
                            ESP_LOGD(TAG, "Queue full, sample dropped");
                            break;
                        }
                    }
                }
            }
        }

        // Report performance every second
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

        // Sleep until next poll interval
        int64_t elapsed = esp_timer_get_time() - loop_start;
        int64_t sleep_time = (FIFO_POLL_INTERVAL_MS * 1000) - elapsed;

        if (sleep_time > 0) {
            vTaskDelay(pdMS_TO_TICKS(sleep_time / 1000));
        } else {
            ESP_LOGW(TAG, "Loop overrun by %lld µs!", -sleep_time);
        }
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
    uint32_t total_written = 0;

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
                // Send raw timestamped samples over BLE
                if (ble_queue != NULL) {
                    // Send batch to BLE queue (we'll update BLE task to handle timestamped samples)
                    for (int i = 0; i < BLE_BATCH_SIZE; i++) {
                        xQueueSend(ble_queue, &ble_batch[i], 0);
                    }
                }
                ble_batch_count = 0;
            }
#endif

            // Write SD batch when full
            if (sd_batch_count >= SD_BATCH_SIZE) {
#ifdef ENABLE_SD_CARD
                if (sd_card != NULL && data_log_file != NULL) {
                    // Write timestamped samples as binary data
                    size_t written = fwrite(sd_batch, sizeof(timestamped_imu_sample_t), SD_BATCH_SIZE, data_log_file);
                    if (written == SD_BATCH_SIZE) {
                        total_written += SD_BATCH_SIZE;
                        ESP_LOGI(TAG, "Wrote %d timestamped samples to SD (total: %lu)", SD_BATCH_SIZE, total_written);
                    } else {
                        ESP_LOGW(TAG, "Failed to write IMU batch to SD card");
                    }
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
                    if (written == sd_batch_count) {
                        total_written += sd_batch_count;
                        ESP_LOGI(TAG, "Wrote partial batch: %d samples (total: %lu)", sd_batch_count, total_written);
                    }
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

        // Wait for samples from BLE queue (with 1 second timeout)
        if (xQueueReceive(ble_queue, &sample, pdMS_TO_TICKS(1000)) == pdTRUE) {
            // Add to BLE batch buffer
            ble_send_batch[ble_batch_count++] = sample;

            // Send when batch is full
            if (ble_batch_count >= BLE_BATCH_SIZE) {
                // Log sample values before sending (occasionally)
                static uint32_t ble_send_count = 0;
                if (ble_send_count++ % 20 == 0) {
                    ESP_LOGI(TAG, "BLE Send - Sample[0]: IMU%d Acc[%d %d %d] Gyro[%d %d %d]",
                             ble_send_batch[0].imu_id,
                             ble_send_batch[0].accel[0], ble_send_batch[0].accel[1], ble_send_batch[0].accel[2],
                             ble_send_batch[0].gyro[0], ble_send_batch[0].gyro[1], ble_send_batch[0].gyro[2]);
                }

                // Send timestamped IMU samples as binary data
                // If BLE is congested, this will fail and drop the batch
                esp_err_t ble_ret = ble_send_timestamped_imu_batch(ble_send_batch, BLE_BATCH_SIZE);

                // Only log success occasionally to reduce console spam
                if (ble_ret == ESP_OK && ble_send_count % 20 == 0) {
                    ESP_LOGI(TAG, "BLE send OK (%d samples, %zu bytes)",
                             BLE_BATCH_SIZE, BLE_BATCH_SIZE * sizeof(timestamped_imu_sample_t));
                }
                // Failures are logged inside ble_send_timestamped_imu_batch() with rate limiting

                ble_batch_count = 0;  // Reset BLE batch counter regardless of success/failure
            }
        }
        // If timeout occurs with no data, just continue waiting
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

                            // Log to SD card if enabled
#ifdef ENABLE_SD_CARD
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
    host.max_freq_khz = SDMMC_FREQ_PROBING;  // Use 400kHz for initialization

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
    snprintf(filepath, sizeof(filepath), "%s/sensor_data.bin", SD_MOUNT_POINT);

    // Try opening with "ab+" which creates the file if it doesn't exist
    // "ab+" = append binary mode, create if doesn't exist
    data_log_file = fopen(filepath, "ab+");
    if (data_log_file == NULL) {
        ESP_LOGE(TAG, "Failed to open data log file: %s", filepath);
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

    ESP_LOGI(TAG, "Opened binary data log: %s", filepath);
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

            // Update connection parameters for high throughput
            esp_ble_conn_update_params_t conn_params = {0};
            memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
            conn_params.latency = 0;
            conn_params.max_int = 0x10;    // 20ms
            conn_params.min_int = 0x06;    // 7.5ms
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
