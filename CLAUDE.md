# CLAUDE.md - AI Assistant Context for Datalogger v2

## Project Overview

This is an ESP32-C6 based datalogger project designed for the **Seeed Studio XIAO ESP32-C6** development board. It collects sensor data from multiple sources and is designed for modular expansion. 
The goal is to collect ans save sensor data as efficiently as possible to be run at as high of a frequency as possible.

### Hardware Platform
- **MCU**: ESP32-C6 (RISC-V based)
- **Development Board**: Seeed Studio XIAO ESP32-C6
- **Framework**: ESP-IDF (Espressif IoT Development Framework)

### Current Status
- GPS module implementation is **working**
- I2C sensors implementation is **disabled** (hardware issues on current board)
- Project is in active development

---

## Architecture

### Modular Design

The project uses **feature flags** for conditional compilation, defined at the top of [main/main.c](main/main.c):

```c
#define ENABLE_I2C_SENSORS    // Enable I2C multiplexer and BMI160 IMU
#define ENABLE_GPS            // Enable GPS module
#define ENABLE_SD_CARD        // Enable SD card logging
```

Comment out these defines to disable specific modules. This allows the code to be compiled for different hardware configurations without modification.

### Hardware Modules

#### 1. GPS Module (GT-U7)
- **Status**: WORKING
- **Interface**: UART1
- **Pins**:
  - RX: GPIO17 (D6) - Receives NMEA data from GPS
  - TX: GPIO16 (D7) - Sends commands to GPS (optional)
- **Baud Rate**: 9600
- **Implementation**: [main/main.c:139-196](main/main.c#L139-L196)
- **Task**: `gps_task()` reads and logs NMEA sentences

#### 2. I2C Sensors (DISABLED)
- **Status**: DISABLED - Hardware issues on current board
- **Sensors**: BMI160 6-axis IMU (accelerometer + gyroscope)
- **Interface**: I2C via PCA9548A multiplexer
- **Pins**:
  - SDA: GPIO22 (D4)
  - SCL: GPIO23 (D5)
- **Implementation**: [main/main.c:71-134](main/main.c#L71-L134)
- **Notes**:
  - Code is complete and tested
  - Re-enable when moving to final hardware with proper I2C connections
  - Multiplexer channel 2 is configured for BMI160

#### 3. SD Card Storage
- **Status**: WORKING
- **Interface**: SPI (SPI2_HOST)
- **Pins**:
  - MOSI: GPIO18 (D10) - Master Out Slave In
  - MISO: GPIO20 (D9) - Master In Slave Out
  - SCK: GPIO19 (D8) - Serial Clock
  - CS: GPIO21 (D3) - Chip Select
- **Mount Point**: `/sdcard`
- **Implementation**: [main/main.c:224-333](main/main.c#L224-L333)
- **Functions**:
  - `sd_card_init()`: Initializes SPI bus and mounts FAT filesystem
  - `sd_card_deinit()`: Safely unmounts and frees resources
  - `sd_write_file()`: Generic file write/append function
  - `sd_log_gps()`: Appends GPS NMEA data to `gps_log.txt`
- **Features**:
  - Automatic GPS data logging when both GPS and SD card are enabled
  - Prints card info (type, capacity) on initialization
  - Graceful degradation if SD card fails to mount

### FreeRTOS Tasks

The application uses FreeRTOS tasks for concurrent operation:

1. **`gps_task`**: Continuously reads UART data from GPS module
2. **`bmi160_test`**: (Disabled) Reads IMU data at 1Hz

Tasks are created in `app_main()` based on enabled feature flags.

---

## Dependencies

### ESP-IDF Component Manager

Dependencies are managed via [main/idf_component.yml](main/idf_component.yml):

```yaml
dependencies:
  idf:
    version: '>=4.1.0'
  esp-idf-lib/bmi160: ^1.0.2    # BMI160 IMU driver
  esp-idf-lib/i2cdev: ^2.0.8    # I2C device abstraction layer
```

Managed components are stored in `managed_components/` (auto-generated, not versioned).

### Key Libraries

1. **esp-idf-lib/bmi160**: BMI160 sensor driver from esp-idf-lib
2. **esp-idf-lib/i2cdev**: I2C device abstraction for easier I2C communication
3. **ESP-IDF Core**: UART, I2C, GPIO, SPI, VFS (Virtual File System), SD card drivers

---

## Pin Mapping (XIAO ESP32-C6)

| XIAO Pin | GPIO | Function      | Module         |
|----------|------|---------------|----------------|
| D3       | 21   | SPI CS        | SD Card        |
| D4       | 22   | I2C SDA       | I2C Sensors    |
| D5       | 23   | I2C SCL       | I2C Sensors    |
| D6       | 17   | UART RX       | GPS            |
| D7       | 16   | UART TX       | GPS (optional) |
| D8       | 20   | SPI SCK       | SD Card        |
| D9       | 18   | SPI MISO      | SD Card        |
| D10      | 19   | SPI MOSI      | SD Card        |

---

## Build System

### Development Environment

The project includes VSCode DevContainer configuration in `.devcontainer/`:
- Pre-configured ESP-IDF environment
- All tools and dependencies included
- Consistent development environment across machines

### Building

```bash
# Configure project (first time only)
idf.py set-target esp32c6

# Build
idf.py build

# Flash and monitor
idf.py flash monitor
```

### Project Structure

```
Datalogger v2/
├── main/
│   ├── main.c              # Main application code
│   ├── CMakeLists.txt      # Component build config
│   └── idf_component.yml   # Component dependencies
├── managed_components/     # Auto-managed dependencies
├── .devcontainer/          # VSCode DevContainer config
├── .vscode/                # VSCode debug config
└── CMakeLists.txt          # Project build config
```

---

## Code Organization

### main.c Structure

The code is organized in clear sections with banner comments:

1. **Feature Flags** (lines 1-6): Enable/disable modules
2. **Common Includes** (lines 8-18): Core ESP-IDF headers
3. **I2C Sensor Configuration** (lines 20-49): I2C pins, addresses, multiplexer config
4. **GPS Configuration** (lines 51-65): UART pins and settings
5. **SD Card Configuration** (lines 67-90): SPI pins and mount settings
6. **I2C Sensor Code** (lines 71-134): BMI160 task implementation
7. **GPS Code** (lines 139-231): GPS UART task implementation with SD logging
8. **SD Card Code** (lines 224-333): SD card initialization and file I/O functions
9. **Main Application** (lines 335-392): Startup, SD init, and task creation

### Logging

The project uses ESP-IDF logging with tag `"DATALOGGER"`:

```c
ESP_LOGI(TAG, "Info message");
ESP_LOGW(TAG, "Warning message");
ESP_LOGE(TAG, "Error message");
```

Filter logs via menuconfig or idf.py monitor.

---

## Known Issues & TODOs

### Current Issues

1. **I2C Sensors Disabled**: Hardware issues on current XIAO board prevent I2C from working reliably
   - **TODO**: Re-enable `ENABLE_I2C_SENSORS` when moving to final hardware
   - Code is complete and tested, just needs working hardware

### Future Enhancements

1. **GPS NMEA Parsing**: Currently logs raw NMEA sentences
   - **TODO**: Parse NMEA to extract lat/lon, altitude, time, etc.
   - Consider using nmea-parser library
   - **TODO**: Add CSV logging with parsed GPS fields (timestamp, lat, lon, speed, etc.)

2. **SD Card Enhancements**:
   - **TODO**: Add IMU data logging when I2C sensors are re-enabled
   - **TODO**: Implement log rotation (new file per day or size limit)
   - **TODO**: Add file listing and management functions
   - **TODO**: Timestamp filenames with GPS time when available

3. **Power Management**: Currently runs continuously
   - **TODO**: Add sleep modes for battery operation
   - **TODO**: GPS power control (sleep when not needed)

4. **Additional Sensors**: Hardware supports more sensors
   - **TODO**: Add temperature/pressure sensors
   - **TODO**: Add analog sensors via ADC

---

## Development Guidelines

### When Adding New Sensors

1. Add feature flag at top of main.c: `#define ENABLE_NEWSENSOR`
2. Add conditional includes and configuration
3. Implement task function with clear logging
4. Create task in `app_main()` with conditional compilation
5. Update this CLAUDE.md file

### Code Style

- Use ESP-IDF logging (ESP_LOGI, etc.) not printf
- Use descriptive variable names
- Add banner comments for major sections
- Keep feature flags at top of file for easy configuration
- Check ESP_ERROR_CHECK for critical operations

### Hardware Configuration

- All pin assignments are in defines at top of each section
- Comment explains XIAO pin mapping (D4, D5, etc.) and GPIO numbers
- I2C frequency is 400kHz (fast mode)
- UART baud rate is 9600 for GPS
- SPI is used for SD card via SPI2_HOST

---

## Testing

### GPS Module

1. Connect GT-U7 GPS module:
   - VCC → 3.3V
   - GND → GND
   - TX → GPIO17 (D6)
   - RX → GPIO16 (D7) - optional
2. Position GPS antenna with clear sky view
3. Flash and monitor: `idf.py flash monitor`
4. Wait 30-60 seconds for cold start (first fix)
5. Should see NMEA sentences like `$GPGGA`, `$GPRMC`, etc.

### SD Card Module

1. Insert FAT32-formatted SD card (or will be auto-formatted)
2. Connect SD card module via SPI:
   - VCC → 3.3V
   - GND → GND
   - MISO → GPIO18 (D9)
   - MOSI → GPIO19 (D10)
   - SCK → GPIO20 (D8)
   - CS → GPIO21 (D3)
3. Enable `#define ENABLE_SD_CARD` (should be enabled by default)
4. Flash and monitor: `idf.py flash monitor`
5. Should see SD card initialization with card type and capacity
6. GPS data (if enabled) will be logged to `/sdcard/gps_log.txt`
7. Access logged files by removing SD card and reading on computer

**Troubleshooting**:
- If SD card fails to mount, check wiring and ensure card is FAT32 formatted
- Try a different SD card if initialization fails
- Check that card is properly inserted
- Verify 3.3V power supply is sufficient (SD cards can draw significant current)

### I2C Sensors (When Re-enabled)

1. Ensure I2C hardware is properly connected
2. Uncomment `#define ENABLE_I2C_SENSORS`
3. Build and flash
4. Should see BMI160 accelerometer and gyroscope data at 1Hz

---

## Git Repository

- **Branch**: master
- **Main Branch**: main (use for PRs)
- **Modified Files** (current session):
  - `main/idf_component.yml` - Dependencies
  - `main/main.c` - Application code
  - `managed_components/esp-idf-lib__bmi160/include/bmi160.h` - Library modifications

---

## Additional Resources

- [ESP-IDF Documentation](https://docs.espressif.com/projects/esp-idf/en/latest/)
- [XIAO ESP32-C6 Wiki](https://wiki.seeedstudio.com/xiao_esp32c6_getting_started/)
- [esp-idf-lib Repository](https://github.com/UncleRus/esp-idf-lib)
- [BMI160 Datasheet](https://www.bosch-sensortec.com/products/motion-sensors/imus/bmi160/)
- [GT-U7 GPS Module Info](https://www.u-blox.com/en/product/neo-7-series)

---

## Quick Reference for AI Assistants

### Common Tasks

**Enable/Disable Modules**: Edit feature flags at top of [main/main.c](main/main.c)

**Add Dependencies**: Edit [main/idf_component.yml](main/idf_component.yml), then run `idf.py reconfigure`

**Change Pin Assignments**: Edit `#define` statements in respective sections of main.c

**Debug Output**: Use `ESP_LOGI(TAG, ...)` for logging, not `printf()`

### Project Context

- This is embedded firmware for resource-constrained MCU
- No dynamic memory allocation in production code (use static allocation)
- FreeRTOS provides multitasking via tasks, not threads
- All hardware access is through ESP-IDF drivers
- Build system is CMake-based (ESP-IDF component system)

### Important Considerations

- **Never use `malloc/free`** in production tasks (use static allocation or FreeRTOS heap)
- **Check return values** with `ESP_ERROR_CHECK()` for critical operations
- **Stack sizes** for tasks are limited - use `configMINIMAL_STACK_SIZE * N`
- **Timing**: Use FreeRTOS `vTaskDelay(pdMS_TO_TICKS(ms))`, not busy loops
- **Concurrency**: Use FreeRTOS queues/semaphores for inter-task communication

---

*Last Updated: 2025-11-26*
*Framework: ESP-IDF v4.1+*
*Target: ESP32-C6*
