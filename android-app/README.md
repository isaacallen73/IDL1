# ESP32 Datalogger Android App

A simple Android app to connect to the ESP32-C6 datalogger via BLE and save GPS/IMU data to CSV files.

## Setup Instructions

### 1. Import into Android Studio

1. Open Android Studio
2. Click "New" > "New Project"
3. Select "Empty Activity" (Compose)
4. Set the following:
   - **Name**: ESP32 Datalogger
   - **Package name**: com.example.esp32datalogger
   - **Language**: Kotlin
   - **Minimum SDK**: API 21 (Android 5.0)
   - **Build configuration language**: Kotlin DSL (build.gradle.kts)
5. Click "Finish"

### 2. Replace Generated Files

After project creation, replace these files with the ones provided:
- `app/src/main/AndroidManifest.xml` (use `AndroidManifest.xml`)
- `app/src/main/java/com/example/esp32datalogger/MainActivity.kt` (use `MainActivity-Compose.kt`)
- `app/build.gradle.kts` (use `build.gradle.kts`)

**Note**: This app uses Jetpack Compose (modern declarative UI). No XML layout files needed.

### 3. Grant Permissions

When you run the app, it will request:
- Bluetooth permissions (BLUETOOTH_SCAN, BLUETOOTH_CONNECT)
- Location permission (required for BLE scanning on Android)
- Storage permissions (for CSV file writing)

Make sure to allow all permissions when prompted.

## How It Works

### BLE Connection

The app scans for a device named "ESP32-Datalogger" and automatically connects when found.

**GATT Service Structure:**
- Service UUID: `0x00FF`
- GPS Characteristic: `0xFF02` (NMEA strings)
- IMU Characteristic: `0xFF01` (binary data)

### Data Logging

**GPS Data** (`gps_log.csv`):
```csv
timestamp,nmea_sentence
2025-11-28 14:23:01.123,$GPGGA,142301.00,3723.2475,N,12158.3416,W,1,07,1.0,9.6,M,-25.6,M,,*65
```

**IMU Data** (`imu_log.csv`):
```csv
timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z
2025-11-28 14:23:01.123,0.05,-0.12,9.81,0.01,-0.02,0.00
```

Files are saved to: `Android/data/com.example.esp32datalogger/files/`

### IMU Data Format

The ESP32 sends binary `imu_sample_t` structures:
```c
typedef struct {
    int64_t timestamp_us;  // 8 bytes - microsecond timestamp
    int16_t accel[3];      // 6 bytes - accelerometer X,Y,Z (raw)
    int16_t gyro[3];       // 6 bytes - gyroscope X,Y,Z (raw)
} imu_sample_t;  // Total: 20 bytes
```

The app converts raw values to physical units:
- Accelerometer: raw / 16384.0 = g (assuming ±2g range)
- Gyroscope: raw / 131.0 = °/s (assuming ±250°/s range)

## Usage

1. Power on the ESP32 datalogger
2. Launch the app
3. Grant all permissions when requested
4. App will automatically scan and connect
5. Press "Start Logging" to begin saving data
6. Press "Stop Logging" to finish and close files
7. CSV files are saved and can be accessed via USB or file manager

## File Locations

CSV files are stored in the app's external files directory:
```
/storage/emulated/0/Android/data/com.example.esp32datalogger/files/
├── gps_log.csv
└── imu_log.csv
```

You can access these files by:
- Connecting phone to PC via USB and browsing with file explorer
- Using Android File Manager app
- Using `adb pull` command

## Troubleshooting

**Can't find ESP32:**
- Make sure Bluetooth is enabled
- Grant location permission (required for BLE scan)
- Ensure ESP32 is powered and advertising

**Connection drops immediately:**
- Check that CCCD descriptors are properly configured on ESP32
- Verify ESP32 firmware compiled successfully with CCCD fixes

**No data in CSV files:**
- Make sure to press "Start Logging" button
- Check that characteristics are sending notifications
- Verify storage permission was granted

## Next Steps

This is a minimal starter app. You can enhance it with:
- NMEA parsing to extract lat/lon/altitude
- Real-time data visualization (graphs)
- Battery level monitoring
- Automatic reconnection on disconnect
- Timestamp synchronization with GPS time
- Data compression for long recordings