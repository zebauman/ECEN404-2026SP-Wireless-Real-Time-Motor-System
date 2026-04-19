package com.remotemotorcontroller.ble

import android.Manifest
import android.annotation.SuppressLint
import android.bluetooth.BluetoothAdapter
import android.bluetooth.BluetoothDevice
import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCallback
import android.bluetooth.BluetoothGattCharacteristic
import android.bluetooth.BluetoothGattDescriptor
import android.bluetooth.BluetoothManager
import android.bluetooth.BluetoothProfile
import android.bluetooth.le.BluetoothLeScanner
import android.bluetooth.le.ScanCallback
import android.bluetooth.le.ScanFilter
import android.bluetooth.le.ScanResult
import android.bluetooth.le.ScanSettings
import android.content.Context
import android.os.Build
import android.os.ParcelUuid
import android.util.Log
import androidx.annotation.RequiresPermission
import androidx.lifecycle.lifecycleScope
import com.remotemotorcontroller.adapter.BleTimeDevice
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.SupervisorJob
import kotlinx.coroutines.delay
import kotlinx.coroutines.flow.MutableSharedFlow
import kotlinx.coroutines.flow.MutableStateFlow
import kotlinx.coroutines.flow.SharedFlow
import kotlinx.coroutines.flow.StateFlow
import kotlinx.coroutines.flow.asStateFlow
import kotlinx.coroutines.isActive
import kotlinx.coroutines.launch
import java.time.Duration
import java.time.Instant
import java.util.UUID

@SuppressLint("StaticFieldLeak")
object BLEManager {

    private val _state = MutableStateFlow<BleState>(BleState.Disconnected)
    val state: StateFlow<BleState> = _state.asStateFlow()

    private lateinit var appCtx: Context
    private lateinit var bluetoothManager: BluetoothManager
    private var bluetoothAdapter: BluetoothAdapter? = null
    private var scanner: BluetoothLeScanner? = null
    private val scannedDevices = mutableListOf<BleTimeDevice>()
    private var isScanning = false
    fun isScanning(): Boolean = isScanning

    // BLE GATT CLIENT -> ALLOWS FOR CONNECTION TO BLE GATT SERVERS
    // INFORMATION REGARDING DISCOVERING SERVICES, READING, AND WRITING CHARACTERISTICS
    private var bluetoothGatt: BluetoothGatt? = null
    private var connectedDevice: BluetoothDevice? = null
    fun getConnectedDevice(): BluetoothDevice? = connectedDevice
    private var userInitDisconnect: Boolean = false

    private var requestQueue: BleRequestQueue? = null
    var activeSession: BleMotorSession? = null
        private set

    // CONFIGURED WITH SETTINGS TO LOCAL VARIABLES
    private var autoReconnectEnabled = true
    private var arCompanyId: Int = 0x706D
    private var arDeviceId: ByteArray? = null
    private var arTimeoutMs = 20_000L
    private var arRetryMs = 500L
    private var filterScanDevice = true // DETERMINE whether to filter the BLE devices when scanning based on service UUID
    private var scanMode: Int = ScanSettings.SCAN_MODE_LOW_LATENCY
    private var cleanupDurationMs: Long = 5_000L

    // LISTENERS
    // FUNCTION TO CALL WHEN A DEVICE IS FOUND
    private var onDeviceFound: ((BleTimeDevice) -> Unit)? = null

    // FUNCTION TO CALL WHEN A DEVICE IS TO BE REMOVED
    private var onDeviceRemoved: ((BleTimeDevice) -> Unit)? = null


    // GATT CHARACTERISTICS
    private var charCmd: BluetoothGattCharacteristic? = null
    private var charTelem: BluetoothGattCharacteristic? = null
    private var charHeartbeat: BluetoothGattCharacteristic? = null

    // JOBS
    // COROUTINE SCOPE TO MANAGE BACKGROUND JOB's LIFECYCLE
    //COROUTINE is A FUNCTION THAT CAN PAUSE AND RESUME ITS EXECUTION WITHOUT BLOCKING THE THREAD
    private val coroutineScope = CoroutineScope(Dispatchers.Main + SupervisorJob())
    private var cleanupJob: Job? = null // PERIODICALLY CLEAN UP THE STALE DEVICES FOR SCANNING
    private var reconnectJob: Job? = null

    private var heartbeatJob: Job? = null

    fun init(context: Context){
        appCtx = context.applicationContext
        bluetoothManager = appCtx.getSystemService(Context.BLUETOOTH_SERVICE) as BluetoothManager
        bluetoothAdapter = bluetoothManager.adapter
        scanner = bluetoothAdapter?.bluetoothLeScanner

        requestQueue = BleRequestQueue(coroutineScope) { bluetoothGatt }
        requestQueue?.start()
    }

    // CALLBACK FUNCTIONS
    // CALLBACK FUNCTION FOR GATT
    private val gattCallback = object : BluetoothGattCallback() {
        // FUNCTION WHEN THE CONNECTION STATE CHANGES OF THE CONNECTED DEVICE -> MANAGED BY GATT
        @SuppressLint("MissingPermission")
        override fun onConnectionStateChange(gatt: BluetoothGatt, status: Int, newState: Int) {
            if(status != BluetoothGatt.GATT_SUCCESS){
                Log.e("BLE", "GATT ERROR $status")
                disconnect()
                return
            }

            if(newState == BluetoothProfile.STATE_CONNECTED){
                _state.value = BleState.Connecting(gatt.device.name)
                reconnectJob?.cancel()
                reconnectJob = null
                gatt.discoverServices()
            }
            else if(newState == BluetoothProfile.STATE_DISCONNECTED){
                _state.value = BleState.Disconnected

                activeSession = null
                gatt.close()
                bluetoothGatt = null
                requestQueue?.clear()
                heartbeatJob?.cancel()

                // AUTO-RECONNECT IFF NOT-USER INIT, ENABLED, AND TARGET ID
                if(!userInitDisconnect && autoReconnectEnabled && arDeviceId?.size == 6){
                    triggerAutoReconnect()
                }
                userInitDisconnect = false
            }
        }
        @SuppressLint("MissingPermission")
        override fun onServicesDiscovered(gatt: BluetoothGatt, status: Int) {
            if(status == BluetoothGatt.GATT_SUCCESS){
                val serv = gatt.getService(BLEContract.SERVICE_MOTOR)
                if(serv == null){
                    Log.e("BLE", "Motor SERVICE NOT FOUND")
                    return
                }
                charCmd = serv.getCharacteristic(BLEContract.CHAR_CMD)
                charTelem = serv.getCharacteristic(BLEContract.CHAR_TELEM)
                charHeartbeat = serv.getCharacteristic(BLEContract.CHAR_HEARTBEAT)

                charTelem?.let{ enableNotifications(gatt, it)}

                requestQueue?.let { queue ->
                    activeSession = BleMotorSession(gatt, queue)
                }

                startHeartbeatLoop()

                _state.value = BleState.Connected(gatt.device.name)
            }else{
                Log.e("BLE", "FAILED TO DISCOVER SERVICES for ${gatt.device?.address}")
            }
        }

        @SuppressLint("MissingPermission")
        override fun onCharacteristicChanged(
            gatt: BluetoothGatt,
            characteristic: BluetoothGattCharacteristic,
            value: ByteArray
        ) {
            val telemetryData = Telemetry.fromBytes(value)

            val currentState = _state.value
            if(currentState is BleState.Connected && telemetryData != null){
                // UPDATE ONLY THE TELEMETRY OF THE STATE
                _state.value = currentState.copy(telemetry = telemetryData)
            }
        }

        override fun onCharacteristicWrite(
            gatt: BluetoothGatt?,
            characteristic: BluetoothGattCharacteristic?,
            status: Int
        ) {
            super.onCharacteristicWrite(gatt, characteristic, status)

            requestQueue?.onWriteComplete()
        }

    }

    // CALLBACK FUNCTION FOR BLE SCAN
    @SuppressLint("MissingPermission")
    private val leScanCallback = object : ScanCallback() {

        // Called when a device is found immediately
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            coroutineScope.launch{ // USING COROUTINE SCOPE HERE TO MAKE SAFE WITH CLEANUP JOB -> NO RACE CONDITIONS
                val device = result.device ?: return@launch

                val existing = scannedDevices.find{ it.bDevice.address == device.address}
                val now = Instant.now()

                if(existing != null){
                    existing.time = now
                    existing.rssi = result.rssi
                    existing.isConnectable = result.isConnectable
                    // INVOKE IS UPDATING THE UI EVERY TIE THE DEVICE IS FOUND/UPDATED -> CALLBACK FUNCTION
                    onDeviceFound?.invoke(existing)
                }else{
                    val id6 = result.scanRecord?.getManufacturerSpecificData(arCompanyId)
                    val newDevice = BleTimeDevice(
                        device,
                        result.rssi,
                        result.isConnectable,
                        now, id6)
                    scannedDevices.add(newDevice)
                    onDeviceFound?.invoke(newDevice)
                }
            }
        }

        override fun onScanFailed(errorCode: Int){
            Log.e("BLE", "Scan failed with error code $errorCode")
        }
    }

    private fun startHeartbeatLoop() {
        heartbeatJob?.cancel() // Safety check
        heartbeatJob = coroutineScope.launch {
            var counter = 0
            while (isActive) {
                // Send heartbeat (incrementing counter or fixed value)
                activeSession?.sendHeartBeat(counter)

                // Increment and wrap around byte size (0-255) if needed
                counter = (counter + 1) % 255

                delay(1000L) // Adjust this interval based on firmware requirements
            }
        }
    }

    // --- SCANNING & CONNECTION ---
    @RequiresPermission(Manifest.permission.BLUETOOTH_SCAN)
    fun startScan() {
        if(isScanning || scanner == null || bluetoothAdapter?.isEnabled != true) return

        // DON'T START MULTIPLE JOBS -> ONLY ONE
        if(cleanupJob?.isActive != true) cleanupJob = startCleanupJob(cleanupDurationMs)

        scannedDevices.clear()

        // SETTINGS FOR THE BLE SCANNER
        val settings = ScanSettings.Builder().setScanMode(
            scanMode).build()

        val filters =
            if (filterScanDevice) listOf(ScanFilter.Builder().
                setServiceUuid(ParcelUuid(BLEContract.SERVICE_MOTOR)).build())
            else emptyList()

        scanner?.startScan(filters, settings, leScanCallback)
        isScanning = true

        if(_state.value is BleState.Disconnected){
            _state.value = BleState.Scanning
        }
    }

    @SuppressLint("MissingPermission")
    fun stopScan() {
        if(!isScanning) return
        scanner?.stopScan(leScanCallback)

        // CANCEL THE JOBS TO STOP THE INFINITE LOOP
        cleanupJob?.cancel()
        cleanupJob = null

        isScanning = false

        if(_state.value is BleState.Scanning){
            _state.value = BleState.Disconnected
        }
    }

    @SuppressLint("MissingPermission")
    fun connect(device: BleTimeDevice){
        stopScan()
        arDeviceId = device.devId
        bluetoothGatt?.close() // CLOSE ANY PREVIOUS CONNECTIONS
        connectedDevice = device.bDevice
        bluetoothGatt = device.bDevice.connectGatt(appCtx, false, gattCallback)
    }

    @SuppressLint("MissingPermission")
    fun disconnect(){
        requestQueue?.clear()

        bluetoothGatt?.disconnect()
        bluetoothGatt?.close()
        _state.value = BleState.Disconnected
        connectedDevice = null
        userInitDisconnect = true
    }


    private fun triggerAutoReconnect() {
        coroutineScope.launch{
            Log.i("BLE", "ATTEMPT RECONNECTION")
            val settings: ScanSettings = ScanSettings.Builder().setScanMode(scanMode).build()

            delay(500)
            stopScan()
            autoReconnect(
                lastDevId48 = arDeviceId!!,
                companyId = arCompanyId,
                serviceUUID = BLEContract.SERVICE_MOTOR,
                timeoutMs = arTimeoutMs,
                retryInterval = arRetryMs,
                scanSettings = settings,
                onTimeout = {} // TODO: ADD TIMEOUT FUNCTIONALITY LIKE POPUP STATING DISCONNECTED
            )
        }
    }

    @SuppressLint("MissingPermission")
    fun autoReconnect(
        lastDevId48: ByteArray,
        companyId: Int,
        serviceUUID: UUID,
        timeoutMs: Long,
        retryInterval: Long,
        scanSettings: ScanSettings,
        onTimeout: () -> Unit
    ){
        require(lastDevId48.size == 6){ "deviceId64 must be 6 Bytes (LE)."}
        if(isScanning()){
            stopScan()
        }

        val mask = ByteArray(6){ 0xFF.toByte() }
        val filters = buildList{
            add(ScanFilter.Builder().setServiceUuid(
                ParcelUuid(serviceUUID)).setManufacturerData(
                companyId, lastDevId48, mask)
                .build())
        }

        reconnectJob = coroutineScope.launch{
            scannedDevices.clear()
            isScanning = true
            scanner?.startScan(filters,scanSettings,leScanCallback)
            val start = System.currentTimeMillis()

            while(System.currentTimeMillis() - start < timeoutMs && isActive){
                val hit = scannedDevices.firstOrNull()

                if(hit != null){
                    Log.i("BLE", "RECONNECTION SUCCESS, HIT DEVICE")
                    stopScan()
                    connect(hit)
                    return@launch
                }
                delay(retryInterval)
            }
            // TIMEOUT
            stopScan()
            onTimeout()
        }
    }

    // --- UTILS ---

    // SETTERS FOR THE LISTENER FUNCTIONS
    fun setDeviceFoundListener(listener: (BleTimeDevice) -> Unit){
        onDeviceFound = listener
    }
    fun setDeviceRemovedListener(listener: (BleTimeDevice) -> Unit){
        onDeviceRemoved = listener
    }

    // HELPER FUNCTION FOR ENABLING NOTIFICATIONS ON THE BLE GATT FOR A CHARACTERISTIC
    @SuppressLint("MissingPermission")
    private fun enableNotifications(gatt: BluetoothGatt, ch: BluetoothGattCharacteristic){
        // CHECK IF THE CHARACTERISTIC HAS THE PROPERTY OF NOTIFY/INDICATE
        if(!gatt.setCharacteristicNotification(ch, true)) return

        val cccd = ch.getDescriptor(BLEContract.DESC_CCCD) ?: return

        val value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.TIRAMISU) {
            gatt.writeDescriptor(cccd, value)
        } else {
            cccd.value = value
            gatt.writeDescriptor(cccd)
        }
    }

    private fun startCleanupJob(
        delayMs: Long = 5_000,

        ) : Job{
        return coroutineScope.launch {
            while (isActive) {
                delay(delayMs)
                val now = Instant.now()
                val timeout = Duration.ofSeconds(10).toMillis()

                val toRemove = mutableListOf<Int>()
                scannedDevices.forEachIndexed { index, device ->
                    val age = Duration.between(device.time, now).toMillis()
                    if (age > timeout) {
                        toRemove.add(index)
                    }
                }
                toRemove.reversed().forEach {

                    onDeviceRemoved?.invoke(scannedDevices[it])
                    scannedDevices.removeAt(it)
                }
            }

        }
    }

    fun applyConfig(
        autoReconnectEnabled: Boolean,
        companyId: Int,
        deviceId: ByteArray?,
        arTimeoutMs: Long,
        arRetryMs: Long,
        scanMode: Int,
        cleanupDurationMs: Long,
        filterScanDevice: Boolean
    ){
        this.autoReconnectEnabled = autoReconnectEnabled
        this.arCompanyId = companyId
        this.arDeviceId = deviceId
        this.arTimeoutMs = arTimeoutMs
        this.arRetryMs = arRetryMs
        this.scanMode = scanMode
        this.cleanupDurationMs = cleanupDurationMs
        this.filterScanDevice = filterScanDevice
    }
}