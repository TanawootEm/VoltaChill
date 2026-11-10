package com.tanawoot.scanlab

import android.Manifest
import android.content.pm.PackageManager
import android.os.Build
import android.os.Bundle
import android.view.KeyEvent
import androidx.core.app.ActivityCompat
import androidx.core.view.ViewCompat
import androidx.core.view.WindowInsetsCompat
import com.google.androidgamesdk.GameActivity

/**
 * Hosts the Rust/egui UI (libvoltachill.so → android_main via GameActivity) and owns the BLE
 * transport (Ble). Requests Bluetooth permissions at startup and routes the system Back
 * button into the Rust navigation stack.
 */
class MainActivity : GameActivity() {
    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        nativeCarMode(BuildConfig.CAR)
        Ble.init(this)
        requestBlePermissions()
        // Keep the screen on while ScanLab is in front: when the screen sleeps the activity
        // pauses and live polling / charge logging stop (long AC-charge recordings).
        window.addFlags(android.view.WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON)

        // Forward the system-bar / cutout insets to Rust so the UI keeps clear of the
        // status bar, the (landscape) navigation bar, and the camera cutout.
        val root = window.decorView
        ViewCompat.setOnApplyWindowInsetsListener(root) { _, insets ->
            val i = insets.getInsets(
                WindowInsetsCompat.Type.systemBars() or WindowInsetsCompat.Type.displayCutout()
            )
            nativeInsets(i.left, i.top, i.right, i.bottom)
            insets
        }
        ViewCompat.requestApplyInsets(root)
    }

    private fun requestBlePermissions() {
        val perms = (if (Build.VERSION.SDK_INT >= 31) {
            listOf(Manifest.permission.BLUETOOTH_SCAN, Manifest.permission.BLUETOOTH_CONNECT)
        } else {
            listOf(Manifest.permission.ACCESS_FINE_LOCATION)
        }) + (if (Build.VERSION.SDK_INT >= 33) listOf(Manifest.permission.POST_NOTIFICATIONS) else emptyList())
        val missing = perms.filter {
            ActivityCompat.checkSelfPermission(this, it) != PackageManager.PERMISSION_GRANTED
        }
        if (missing.isNotEmpty()) {
            ActivityCompat.requestPermissions(this, missing.toTypedArray(), 1)
        }
    }

    // GameActivity delivers the system Back here. Pop one in-app screen; at the root, send the
    // task to the background instead of finish() — finishing destroys the Activity while the
    // native thread survives, so reopening would show a black screen.
    override fun onKeyDown(keyCode: Int, event: KeyEvent?): Boolean {
        if (keyCode == KeyEvent.KEYCODE_BACK) {
            if (nativeBack()) return true   // popped a screen
            moveTaskToBack(true)            // at the root -> minimise, keep the app alive
            return true
        }
        return super.onKeyDown(keyCode, event)
    }

    /** Implemented in Rust (app.rs androidback): true if a screen was popped, false at root. */
    external fun nativeBack(): Boolean

    /** Implemented in Rust (app.rs androidback): system-bar/cutout insets in physical px. */
    external fun nativeInsets(l: Int, t: Int, r: Int, b: Int)

    /** Implemented in Rust (app.rs androidback): car flavor = smaller UI + bottom bar room. */
    external fun nativeCarMode(on: Boolean)
}
