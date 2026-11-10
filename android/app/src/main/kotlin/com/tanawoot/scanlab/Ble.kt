package com.tanawoot.scanlab

import android.annotation.SuppressLint
import android.bluetooth.*
import android.bluetooth.le.*
import android.content.Context
import android.os.Build
import android.os.Handler
import android.os.Looper
import android.util.Log
import java.util.UUID
import java.util.concurrent.Semaphore
import java.util.concurrent.TimeUnit

/**
 * BLE transport for the ELM327 OBD adapter (service FFF0, write FFF2, notify FFF1).
 *
 * Rust calls the @JvmStatic command methods (startScan/connect/write/...). This object calls
 * back into Rust with the external native methods (nativeOn*). All the ELM327/UDS protocol
 * logic stays in Rust (obd.rs); only the raw BLE bytes cross JNI.
 */
@SuppressLint("MissingPermission")
object Ble {
    private const val TAG = "ScanLabBle"
    private val SVC: UUID = UUID.fromString("0000fff0-0000-1000-8000-00805f9b34fb")
    private val WRITE: UUID = UUID.fromString("0000fff2-0000-1000-8000-00805f9b34fb")
    private val NOTIFY: UUID = UUID.fromString("0000fff1-0000-1000-8000-00805f9b34fb")
    private val CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    private var ctx: Context? = null
    private var adapter: BluetoothAdapter? = null
    private var scanner: BluetoothLeScanner? = null
    private val main = Handler(Looper.getMainLooper())

    private var gatt: BluetoothGatt? = null
    private var writeChar: BluetoothGattCharacteristic? = null
    private val seen = HashMap<String, Boolean>()
    private var scanning = false

    fun init(context: Context) {
        ctx = context.applicationContext
        val bm = context.getSystemService(Context.BLUETOOTH_SERVICE) as? BluetoothManager
        adapter = bm?.adapter
        nativeInit()
    }

    // ---- commands called from Rust -------------------------------------------------

    @JvmStatic
    fun startScan() {
        val a = adapter
        if (a == null || !a.isEnabled) { nativeOnScanDone(); return }
        scanner = a.bluetoothLeScanner
        if (scanner == null) { nativeOnScanDone(); return }
        seen.clear()
        scanning = true
        val settings = ScanSettings.Builder()
            .setScanMode(ScanSettings.SCAN_MODE_LOW_LATENCY)
            .build()
        try {
            scanner?.startScan(null, settings, scanCb)
            Log.i(TAG, "scan started")
        } catch (e: Exception) {
            Log.e(TAG, "startScan: $e"); scanning = false; nativeOnScanDone(); return
        }
        // stop after 6 s
        main.postDelayed({ stopScan() }, 6000)
    }

    @JvmStatic
    fun stopScan() {
        if (!scanning) return
        scanning = false
        try { scanner?.stopScan(scanCb) } catch (_: Exception) {}
        Log.i(TAG, "scan stopped -> nativeOnScanDone")
        nativeOnScanDone()
    }

    @JvmStatic
    fun connect(id: String) {
        val a = adapter ?: run { nativeOnDisconnected("no adapter"); return }
        val dev = try { a.getRemoteDevice(id) } catch (e: Exception) { nativeOnDisconnected("bad id"); return }
        stopScan()
        main.post {
            gatt = dev.connectGatt(ctx, false, gattCb, BluetoothDevice.TRANSPORT_LE)
        }
    }

    /**
     * Android GATT runs ONE operation at a time: a write issued before the previous one's
     * callback is silently dropped ("prior command is not finished"). Every GATT op takes this
     * permit; the matching callback (onCharacteristicWrite / onDescriptorWrite) gives it back.
     */
    private val gattOp = Semaphore(1)

    private fun takeOp() {
        // A lost callback must not wedge the link: after 2 s proceed anyway.
        if (!gattOp.tryAcquire(2, TimeUnit.SECONDS)) Log.w(TAG, "GATT op permit timeout, continuing")
    }

    private fun giveOp() {
        if (gattOp.availablePermits() == 0) gattOp.release()
    }

    private fun resetOp() {
        gattOp.drainPermits(); gattOp.release()
    }

    /** Called from the Rust worker thread (never the main thread), so it may block. */
    @JvmStatic
    fun write(data: ByteArray) {
        val g = gatt ?: return
        val c = writeChar ?: return
        takeOp()
        var ok = false
        for (attempt in 0 until 5) {
            ok = try {
                if (Build.VERSION.SDK_INT >= 33) {
                    g.writeCharacteristic(c, data, BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT) ==
                        BluetoothStatusCodes.SUCCESS
                } else {
                    @Suppress("DEPRECATION")
                    c.value = data
                    @Suppress("DEPRECATION")
                    c.writeType = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT
                    @Suppress("DEPRECATION")
                    g.writeCharacteristic(c)
                }
            } catch (e: Exception) { Log.e(TAG, "write: $e"); false }
            if (ok) break
            Thread.sleep(30) // stack still busy: retry briefly
        }
        if (!ok) { Log.e(TAG, "write dropped after retries"); giveOp() }
    }

    /** Background monitoring on: foreground service + wake lock (see MonitorService). */
    @JvmStatic
    fun startMonitor() {
        val c = ctx ?: return
        main.post {
            try {
                val i = android.content.Intent(c, MonitorService::class.java)
                if (Build.VERSION.SDK_INT >= 26) c.startForegroundService(i) else c.startService(i)
            } catch (e: Exception) { Log.e(TAG, "startMonitor: $e") }
        }
    }

    @JvmStatic
    fun stopMonitor() {
        val c = ctx ?: return
        main.post {
            try { c.stopService(android.content.Intent(c, MonitorService::class.java)) } catch (_: Exception) {}
        }
    }

    @JvmStatic
    fun disconnect() {
        main.post {
            try { gatt?.disconnect(); gatt?.close() } catch (_: Exception) {}
            gatt = null; writeChar = null
        }
    }

    // ---- BLE callbacks -------------------------------------------------------------

    private val scanCb = object : ScanCallback() {
        override fun onScanResult(callbackType: Int, result: ScanResult) {
            val dev = result.device
            val addr = dev.address ?: return
            // Prefer the advertised local name; fall back to the GATT name.
            val name = result.scanRecord?.deviceName
                ?: (try { dev.name } catch (e: Exception) { null }) ?: ""
            val uuids = result.scanRecord?.serviceUuids
            val up = name.uppercase()
            val isObd = (uuids?.any { it.uuid == SVC } == true) ||
                listOf("OBD", "ELM", "VLINK", "V-LINK", "VEEPEAK", "KONNWEI").any { up.contains(it) }
            // Declutter: only surface devices that have a name or look like an OBD adapter.
            if (name.isEmpty() && !isObd) return
            val had = seen[addr]
            if (had == true) return                 // already reported with a name
            seen[addr] = name.isNotEmpty()
            if (isObd) Log.i(TAG, "OBD device $addr '$name'")
            nativeOnDevice(addr, name, isObd)
        }
        override fun onScanFailed(errorCode: Int) {
            Log.e(TAG, "scan failed $errorCode"); scanning = false; nativeOnScanDone()
        }
    }

    private val gattCb = object : BluetoothGattCallback() {
        override fun onConnectionStateChange(g: BluetoothGatt, status: Int, newState: Int) {
            if (newState == BluetoothProfile.STATE_CONNECTED) {
                main.post { g.discoverServices() }
            } else if (newState == BluetoothProfile.STATE_DISCONNECTED) {
                nativeOnDisconnected("disconnected ($status)")
                try { g.close() } catch (_: Exception) {}
                if (gatt === g) { gatt = null; writeChar = null }
                resetOp()
            }
        }

        override fun onServicesDiscovered(g: BluetoothGatt, status: Int) {
            val svc = g.getService(SVC)
                ?: g.services.firstOrNull { it.getCharacteristic(WRITE) != null && it.getCharacteristic(NOTIFY) != null }
            if (svc == null) { nativeOnDisconnected("no FFF0 service"); return }
            writeChar = svc.getCharacteristic(WRITE) ?: svc.characteristics.firstOrNull {
                (it.properties and (BluetoothGattCharacteristic.PROPERTY_WRITE or BluetoothGattCharacteristic.PROPERTY_WRITE_NO_RESPONSE)) != 0
            }
            val notifyChar = svc.getCharacteristic(NOTIFY) ?: svc.characteristics.firstOrNull {
                (it.properties and BluetoothGattCharacteristic.PROPERTY_NOTIFY) != 0
            }
            if (writeChar == null || notifyChar == null) { nativeOnDisconnected("no rw chars"); return }
            g.setCharacteristicNotification(notifyChar, true)
            resetOp()
            val cccd = notifyChar.getDescriptor(CCCD)
            if (cccd == null) { nativeOnConnected(g.device.name ?: g.device.address); return }
            takeOp()
            if (Build.VERSION.SDK_INT >= 33) {
                g.writeDescriptor(cccd, BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE)
            } else {
                @Suppress("DEPRECATION")
                cccd.value = BluetoothGattDescriptor.ENABLE_NOTIFICATION_VALUE
                @Suppress("DEPRECATION")
                g.writeDescriptor(cccd)
            }
            // nativeOnConnected follows in onDescriptorWrite, so the first ELM command (ATZ)
            // can't collide with this descriptor write.
        }

        override fun onDescriptorWrite(g: BluetoothGatt, d: BluetoothGattDescriptor, status: Int) {
            giveOp()
            if (d.uuid == CCCD) nativeOnConnected(g.device.name ?: g.device.address)
        }

        override fun onCharacteristicWrite(g: BluetoothGatt, c: BluetoothGattCharacteristic, status: Int) {
            if (status != BluetoothGatt.GATT_SUCCESS) Log.w(TAG, "write status $status")
            giveOp()
        }

        @Deprecated("Deprecated in Java")
        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic) {
            @Suppress("DEPRECATION")
            nativeOnNotify(c.value ?: ByteArray(0))
        }

        override fun onCharacteristicChanged(g: BluetoothGatt, c: BluetoothGattCharacteristic, value: ByteArray) {
            nativeOnNotify(value)
        }
    }

    // ---- native callbacks implemented in Rust (obd.rs) -----------------------------
    external fun nativeInit()
    external fun nativeOnDevice(id: String, name: String, isObd: Boolean)
    external fun nativeOnScanDone()
    external fun nativeOnConnected(info: String)
    external fun nativeOnDisconnected(reason: String)
    external fun nativeOnNotify(data: ByteArray)
}
