package com.example.esp32datalogger

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.Context
import android.content.pm.PackageManager
import android.location.LocationManager
import android.os.Build
import android.os.Bundle
import android.os.Handler
import android.os.Looper
import android.util.Log
import android.widget.Toast
import androidx.activity.ComponentActivity
import androidx.activity.compose.setContent
import androidx.activity.result.contract.ActivityResultContracts
import androidx.compose.foundation.background
import androidx.compose.foundation.layout.*
import androidx.compose.material3.*
import androidx.compose.runtime.*
import androidx.compose.ui.Alignment
import androidx.compose.ui.Modifier
import androidx.compose.ui.graphics.Color
import androidx.compose.ui.graphics.toArgb
import androidx.compose.ui.text.font.FontWeight
import androidx.compose.ui.unit.dp
import androidx.compose.ui.unit.sp
import androidx.core.content.ContextCompat
import com.patrykandpatrick.vico.compose.axis.horizontal.rememberBottomAxis
import com.patrykandpatrick.vico.compose.axis.vertical.rememberStartAxis
import com.patrykandpatrick.vico.compose.chart.Chart
import com.patrykandpatrick.vico.compose.chart.line.lineChart
import com.patrykandpatrick.vico.core.axis.AxisItemPlacer
import com.patrykandpatrick.vico.core.chart.line.LineChart
import com.patrykandpatrick.vico.core.chart.values.AxisValuesOverrider
import com.patrykandpatrick.vico.core.entry.ChartEntryModelProducer
import com.patrykandpatrick.vico.core.entry.entryOf
import java.io.File
import java.io.FileWriter
import java.nio.ByteBuffer
import java.nio.ByteOrder
import java.text.SimpleDateFormat
import java.util.*

class MainActivity : ComponentActivity() {

    companion object {
        private const val TAG = "ESP32Datalogger"
        private const val DEVICE_NAME = "ESP32-Datalogger"
        private val SERVICE_UUID = UUID.fromString("000000FF-0000-1000-8000-00805F9B34FB")
        private val GPS_CHAR_UUID = UUID.fromString("0000FF02-0000-1000-8000-00805F9B34FB")
        private val IMU_CHAR_UUID = UUID.fromString("0000FF01-0000-1000-8000-00805F9B34FB")
        private val CCCD_UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")
        private const val MAX_CHART_ENTRIES = 200
        private const val DOWNSAMPLE_FACTOR = 10
    }

    private var statusText by mutableStateOf("Requesting permissions...")
    private var dataText by mutableStateOf("No data yet...")
    private var isConnected by mutableStateOf(false)
    private var isLogging by mutableStateOf(false)
    private var isScanning by mutableStateOf(false)

    private var bluetoothAdapter: BluetoothAdapter? = null
    private var bleScanner: BluetoothLeScanner? = null
    private var bluetoothGatt: BluetoothGatt? = null

    private var gpsLogFile: File? = null
    private var imuLogFile: File? = null
    private var gpsWriter: FileWriter? = null
    private var imuWriter: FileWriter? = null
    private val timestampFormat = SimpleDateFormat("yyyy-MM-dd HH:mm:ss.SSS", Locale.US)

    private val mainThreadHandler = Handler(Looper.getMainLooper())

    // Chart data
    private val xAccelModelProducer = ChartEntryModelProducer()
    private val yAccelModelProducer = ChartEntryModelProducer()
    private val zAccelModelProducer = ChartEntryModelProducer()
    private val xAccelEntries = mutableListOf<com.patrykandpatrick.vico.core.entry.FloatEntry>()
    private val yAccelEntries = mutableListOf<com.patrykandpatrick.vico.core.entry.FloatEntry>()
    private val zAccelEntries = mutableListOf<com.patrykandpatrick.vico.core.entry.FloatEntry>()
    private var chartEntryX = 0f
    private var imuSampleCounter = 0

    private val permissionLauncher = registerForActivityResult(
        ActivityResultContracts.RequestMultiplePermissions()
    ) { permissions ->
        if (permissions.all { it.value }) {
            mainThreadHandler.post { statusText = "Ready to connect" }
        } else {
            mainThreadHandler.post { statusText = "ERROR: Permissions required" }
            Toast.makeText(this, "All permissions required for BLE and storage", Toast.LENGTH_LONG).show()
        }
    }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        val bluetoothManager = getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter
        requestPermissions()

        setContent {
            MaterialTheme {
                Surface(modifier = Modifier.fillMaxSize(), color = MaterialTheme.colorScheme.background) {
                    DataloggerScreen()
                }
            }
        }
    }

    @Composable
    fun DataloggerScreen() {
        Column(
            modifier = Modifier.fillMaxSize().padding(16.dp),
            horizontalAlignment = Alignment.CenterHorizontally
        ) {
            Spacer(modifier = Modifier.height(32.dp))
            Text(text = "ESP32 Datalogger", fontSize = 24.sp, fontWeight = FontWeight.Bold)
            Spacer(modifier = Modifier.height(24.dp))
            Text(
                text = statusText,
                modifier = Modifier.fillMaxWidth().background(Color(0xFFE0E0E0)).padding(12.dp),
                fontSize = 16.sp
            )
            Spacer(modifier = Modifier.height(24.dp))
            Button(
                onClick = { toggleConnection() },
                modifier = Modifier.fillMaxWidth().height(56.dp),
                enabled = !isScanning
            ) {
                Text(
                    text = when {
                        isScanning -> "Scanning..."
                        isConnected -> "Disconnect"
                        else -> "Connect"
                    },
                    fontSize = 18.sp
                )
            }
            Spacer(modifier = Modifier.height(16.dp))
            Button(
                onClick = { toggleLogging() },
                modifier = Modifier.fillMaxWidth().height(56.dp),
                enabled = isConnected
            ) {
                Text(
                    text = if (isLogging) "Stop Logging" else "Start Logging",
                    fontSize = 18.sp
                )
            }
            Spacer(modifier = Modifier.height(32.dp))
            Text(text = "IMU Accelerometer Data", fontSize = 14.sp, fontWeight = FontWeight.Bold)
            Spacer(modifier = Modifier.height(8.dp))
            IMUChart(modelProducer = xAccelModelProducer, name = "X-Axis", color = Color.Red)
            IMUChart(modelProducer = yAccelModelProducer, name = "Y-Axis", color = Color.Green)
            IMUChart(modelProducer = zAccelModelProducer, name = "Z-Axis", color = Color.Blue)
            Spacer(modifier = Modifier.height(16.dp))
            Text(
                text = "CSV files saved to:\nAndroid/data/com.example.esp32datalogger/files/",
                fontSize = 10.sp,
                color = Color.Gray
            )
        }
    }

    @Composable
    fun IMUChart(modelProducer: ChartEntryModelProducer, name: String, color: Color) {
        Column(modifier = Modifier.fillMaxWidth().height(120.dp)) {
            Text(name)
            Chart(
                chart = lineChart(
                    lines = listOf(LineChart.LineSpec(lineColor = color.toArgb())),
                    axisValuesOverrider = AxisValuesOverrider.fixed(minY = -2.5f, maxY = 2.5f)
                ),
                chartModelProducer = modelProducer,
                startAxis = rememberStartAxis(
                    itemPlacer = AxisItemPlacer.Vertical.default(maxItemCount = 5),
                    valueFormatter = { value, _ -> "%.1f".format(value) }
                ),
                bottomAxis = rememberBottomAxis(),
                modifier = Modifier.fillMaxSize()
            )
        }
    }

    private fun requestPermissions() {
        val permissions = if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.S) {
            arrayOf(
                Manifest.permission.BLUETOOTH_SCAN,
                Manifest.permission.BLUETOOTH_CONNECT,
                Manifest.permission.ACCESS_FINE_LOCATION
            )
        } else {
            arrayOf(
                Manifest.permission.BLUETOOTH,
                Manifest.permission.BLUETOOTH_ADMIN,
                Manifest.permission.ACCESS_FINE_LOCATION,
                Manifest.permission.WRITE_EXTERNAL_STORAGE
            )
        }
        if (permissions.all { ContextCompat.checkSelfPermission(this, it) == PackageManager.PERMISSION_GRANTED }) {
            mainThreadHandler.post { statusText = "Ready to connect" }
        } else {
            permissionLauncher.launch(permissions)
        }
    }

    @SuppressLint("MissingPermission")
    private fun toggleConnection() {
        if (isConnected) {
            disconnect()
        } else {
            startScan()
        }
    }

    @SuppressLint("MissingPermission")
    private fun startScan() {
        val locationManager = getSystemService(Context.LOCATION_SERVICE) as LocationManager
        if (!locationManager.isProviderEnabled(LocationManager.GPS_PROVIDER) && !locationManager.isProviderEnabled(LocationManager.NETWORK_PROVIDER)) {
            mainThreadHandler.post {
                statusText = "ERROR: Please enable Location Services"
                Toast.makeText(this, "Location Services must be enabled for BLE scanning", Toast.LENGTH_LONG).show()
            }
            return
        }

        bleScanner = bluetoothAdapter?.bluetoothLeScanner
        if (bleScanner == null) {
            mainThreadHandler.post { statusText = "ERROR: Bluetooth not available" }
            return
        }

        mainThreadHandler.post {
            statusText = "Scanning for $DEVICE_NAME..."
            isScanning = true
        }

        val scanFilter = ScanFilter.Builder().setDeviceName(DEVICE_NAME).build()
        val filters = listOf(scanFilter)

        val scanSettings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .setLegacy(false)
            .setPhy(ScanSettings.PHY_LE_ALL_SUPPORTED)
            .build()

        bleScanner?.startScan(filters, scanSettings, scanCallback)

        mainThreadHandler.postDelayed({
            if (isScanning) {
                stopScan()
                mainThreadHandler.post { statusText = "Scan timeout - device not found" }
            }
        }, 10000)
    }

    @SuppressLint("MissingPermission")
    private fun stopScan() {
        if (isScanning) {
            bleScanner?.stopScan(scanCallback)
            mainThreadHandler.post { isScanning = false }
        }
    }

    private val scanCallback = object : ScanCallback() {
        @SuppressLint("MissingPermission")
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            if (result.device.name == DEVICE_NAME) {
                Log.d(TAG, "Found $DEVICE_NAME: ${result.device.address}")
                stopScan()
                mainThreadHandler.post { statusText = "Found device - connecting..." }
                connectToDevice(result.device)
            }
        }

        override fun onScanFailed(errorCode: Int) {
            Log.e(TAG, "Scan failed: $errorCode")
            mainThreadHandler.post {
                statusText = "ERROR: Scan failed ($errorCode)"
                isScanning = false
            }
        }
    }

    @SuppressLint("MissingPermission")
    private fun connectToDevice(device: BluetoothDevice) {
        mainThreadHandler.post { statusText = "Connecting to ${device.address}..." }
        bluetoothGatt = device.connectGatt(this, false, gattCallback)
    }

    @SuppressLint("MissingPermission")
    private fun disconnect() {
        stopLogging()
        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        bluetoothGatt = null
        mainThreadHandler.post {
            isConnected = false
            statusText = "Disconnected"
            // Clear chart data on disconnect
            chartEntryX = 0f
            xAccelEntries.clear()
            yAccelEntries.clear()
            zAccelEntries.clear()
            xAccelModelProducer.setEntries(emptyList<com.patrykandpatrick.vico.core.entry.FloatEntry>())
            yAccelModelProducer.setEntries(emptyList<com.patrykandpatrick.vico.core.entry.FloatEntry>())
            zAccelModelProducer.setEntries(emptyList<com.patrykandpatrick.vico.core.entry.FloatEntry>())
        }
    }

    private val gattCallback = object : BluetoothGattCallback() {
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            mainThreadHandler.post {
                if (newState == BluetoothProfile.STATE_CONNECTED) {
                    Log.d(TAG, "Connected to GATT server.")
                    statusText = "Connected - discovering services..."
                    gatt.discoverServices()
                } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                    Log.d(TAG, "Disconnected from GATT server.")
                    isConnected = false
                    statusText = "Disconnected"
                }
            }
        }

        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            mainThreadHandler.post {
                if (status == BluetoothGatt.GATT_SUCCESS) {
                    val service = gatt.getService(SERVICE_UUID)
                    if (service != null) {
                        Log.d(TAG, "Found datalogger service")
                        val gpsChar = service.getCharacteristic(GPS_CHAR_UUID)
                        val imuChar = service.getCharacteristic(IMU_CHAR_UUID)
                        gpsChar?.let { enableNotifications(gatt, it) }
                        imuChar?.let { enableNotifications(gatt, it) }
                        isConnected = true
                        statusText = "Connected - ready to log"
                    } else {
                        Log.e(TAG, "Datalogger service not found")
                        statusText = "ERROR: Datalogger service not found"
                        gatt.disconnect()
                    }
                } else {
                    Log.w(TAG, "onServicesDiscovered received: $status")
                }
            }
        }

        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray
        ) {
            when (characteristic.uuid) {
                GPS_CHAR_UUID -> handleGpsData(value)
                IMU_CHAR_UUID -> handleImuData(value)
            }
        }
    }

    @SuppressLint("MissingPermission")
    private fun enableNotifications(gatt: BluetoothGatt, characteristic: BluetoothGattCharacteristic) {
        gatt.setCharacteristicNotification(characteristic, true)
        val descriptor = characteristic.getDescriptor(CCCD_UUID)
        descriptor?.let {
            it.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
            gatt.writeDescriptor(it)
            Log.d(TAG, "Enabled notifications for ${characteristic.uuid}")
        }
    }

    private fun handleGpsData(data: ByteArray) {
        val nmea = String(data, Charsets.UTF_8).trim()
        Log.d(TAG, "GPS: $nmea")
        mainThreadHandler.post { dataText = "GPS: $nmea" }
        if (isLogging && gpsWriter != null) {
            try {
                val timestamp = timestampFormat.format(Date())
                gpsWriter?.append("$timestamp,$nmea\n")
                gpsWriter?.flush()
            } catch (e: Exception) {
                Log.e(TAG, "Failed to write GPS data", e)
            }
        }
    }

    private fun handleImuData(data: ByteArray) {
        if (data.size < 20) {
            Log.w(TAG, "IMU data too short: ${data.size} bytes")
            return
        }
        val buffer = ByteBuffer.wrap(data).order(ByteOrder.LITTLE_ENDIAN)
        buffer.long // Skip timestamp
        val accelX = buffer.short / 16384.0f
        val accelY = buffer.short / 16384.0f
        val accelZ = buffer.short / 16384.0f
        val gyroX = buffer.short / 131.0f
        val gyroY = buffer.short / 131.0f
        val gyroZ = buffer.short / 131.0f

        // Downsample for chart rendering
        imuSampleCounter++
        if (imuSampleCounter % DOWNSAMPLE_FACTOR == 0) {
            mainThreadHandler.post {
                if (xAccelEntries.size >= MAX_CHART_ENTRIES) xAccelEntries.removeAt(0)
                if (yAccelEntries.size >= MAX_CHART_ENTRIES) yAccelEntries.removeAt(0)
                if (zAccelEntries.size >= MAX_CHART_ENTRIES) zAccelEntries.removeAt(0)

                xAccelEntries.add(entryOf(chartEntryX, accelX))
                yAccelEntries.add(entryOf(chartEntryX, accelY))
                zAccelEntries.add(entryOf(chartEntryX, accelZ))

                xAccelModelProducer.setEntries(xAccelEntries)
                yAccelModelProducer.setEntries(yAccelEntries)
                zAccelModelProducer.setEntries(zAccelEntries)

                chartEntryX++
            }
        }

        if (isLogging && imuWriter != null) {
            try {
                val timestamp = timestampFormat.format(Date())
                imuWriter?.append("$timestamp,${accelX},${accelY},${accelZ},${gyroX},${gyroY},${gyroZ}\n")
                imuWriter?.flush()
            } catch (e: Exception) {
                Log.e(TAG, "Failed to write IMU data", e)
            }
        }
    }

    private fun toggleLogging() {
        if (isLogging) {
            stopLogging()
        } else {
            startLogging()
        }
    }

    private fun startLogging() {
        try {
            val filesDir = getExternalFilesDir(null) ?: return
            val timestamp = SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(Date())
            gpsLogFile = File(filesDir, "gps_log_$timestamp.csv")
            imuLogFile = File(filesDir, "imu_log_$timestamp.csv")
            gpsWriter = FileWriter(gpsLogFile, false)
            imuWriter = FileWriter(imuLogFile, false)
            gpsWriter?.append("timestamp,nmea_sentence\n")
            imuWriter?.append("timestamp,accel_x,accel_y,accel_z,gyro_x,gyro_y,gyro_z\n")
            mainThreadHandler.post {
                isLogging = true
                statusText = "Logging to files..."
            }
            Toast.makeText(this, "Logging started", Toast.LENGTH_SHORT).show()
        } catch (e: Exception) {
            Log.e(TAG, "Failed to start logging", e)
            Toast.makeText(this, "Failed to start logging: ${e.message}", Toast.LENGTH_LONG).show()
        }
    }

    private fun stopLogging() {
        if (isLogging) {
            try {
                gpsWriter?.close()
                imuWriter?.close()
                gpsWriter = null
                imuWriter = null
                mainThreadHandler.post {
                    isLogging = false
                    statusText = if (isConnected) "Connected - ready to log" else "Disconnected"
                }
                val message = "Saved:\n${gpsLogFile?.name}\n${imuLogFile?.name}"
                Toast.makeText(this, message, Toast.LENGTH_LONG).show()
            } catch (e: Exception) {
                Log.e(TAG, "Failed to stop logging", e)
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()
        disconnect()
    }
}
