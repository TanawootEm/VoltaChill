package com.tanawoot.scanlab

import android.app.Notification
import android.app.NotificationChannel
import android.app.NotificationManager
import android.app.PendingIntent
import android.app.Service
import android.content.Intent
import android.content.pm.ServiceInfo
import android.os.Build
import android.os.IBinder
import android.os.PowerManager
import android.util.Log

/**
 * Foreground service that keeps VoltaChill alive while it monitors a charge with the screen off
 * or the app in the background: an ongoing notification (required) and a partial wake lock so
 * the CPU — and with it the Rust OBD worker and the BLE link — keeps running. It does no work
 * itself; the worker thread does the reading / recording (Cmd::BgMonitor).
 * Started / stopped from Rust via Ble.startMonitor() / Ble.stopMonitor().
 */
class MonitorService : Service() {
    private var wake: PowerManager.WakeLock? = null

    override fun onBind(intent: Intent?): IBinder? = null

    override fun onStartCommand(intent: Intent?, flags: Int, startId: Int): Int {
        val nm = getSystemService(NOTIFICATION_SERVICE) as NotificationManager
        if (Build.VERSION.SDK_INT >= 26) {
            nm.createNotificationChannel(NotificationChannel(CHANNEL, "VoltaChill monitoring", NotificationManager.IMPORTANCE_LOW))
        }
        val open = PendingIntent.getActivity(
            this, 0, Intent(this, MainActivity::class.java).addFlags(Intent.FLAG_ACTIVITY_SINGLE_TOP),
            PendingIntent.FLAG_IMMUTABLE or PendingIntent.FLAG_UPDATE_CURRENT
        )
        val b = if (Build.VERSION.SDK_INT >= 26) Notification.Builder(this, CHANNEL) else @Suppress("DEPRECATION") Notification.Builder(this)
        val n = b.setContentTitle("VoltaChill")
            .setContentText("กำลังบันทึก / ควบคุมระหว่างชาร์จ")
            .setSmallIcon(R.mipmap.ic_launcher)
            .setContentIntent(open)
            .setOngoing(true)
            .build()
        try {
            if (Build.VERSION.SDK_INT >= 29) {
                startForeground(ID, n, ServiceInfo.FOREGROUND_SERVICE_TYPE_CONNECTED_DEVICE)
            } else {
                startForeground(ID, n)
            }
        } catch (e: Exception) {
            Log.e(TAG, "startForeground: $e"); stopSelf(); return START_NOT_STICKY
        }
        if (wake == null) {
            val pm = getSystemService(POWER_SERVICE) as PowerManager
            wake = pm.newWakeLock(PowerManager.PARTIAL_WAKE_LOCK, "VoltaChill:monitor").apply {
                setReferenceCounted(false)
                acquire(12 * 60 * 60 * 1000L) // safety cap: 12 h
            }
        }
        Log.i(TAG, "monitor service started")
        return START_NOT_STICKY
    }

    override fun onDestroy() {
        wake?.let { if (it.isHeld) it.release() }
        wake = null
        Log.i(TAG, "monitor service stopped")
        super.onDestroy()
    }

    companion object {
        private const val TAG = "VoltaChillMonitor"
        private const val CHANNEL = "monitor"
        private const val ID = 7
    }
}
