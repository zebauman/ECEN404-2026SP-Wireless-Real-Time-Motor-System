package com.remotemotorcontroller.ui

import android.os.Bundle
import androidx.activity.viewModels
import androidx.appcompat.app.AppCompatActivity
import androidx.core.view.isVisible
import androidx.fragment.app.activityViewModels
import androidx.lifecycle.Lifecycle
import androidx.lifecycle.lifecycleScope
import androidx.lifecycle.repeatOnLifecycle
import androidx.navigation.fragment.NavHostFragment
import androidx.navigation.ui.setupWithNavController
import com.google.android.material.bottomnavigation.BottomNavigationView
import com.remotemotorcontroller.App
import com.remotemotorcontroller.R
import com.remotemotorcontroller.adapter.AnalyticsViewModel
import com.remotemotorcontroller.ble.BLEManager
import com.remotemotorcontroller.ble.BleState
import com.remotemotorcontroller.ui.widgets.DeviceHeader
import com.remotemotorcontroller.ui.widgets.LiveSummaryView
import kotlinx.coroutines.launch
import kotlin.getValue

class ShellActivity : AppCompatActivity() {

    private lateinit var deviceHeader: DeviceHeader
    private lateinit var liveSummary: LiveSummaryView

    private val viewModel: AnalyticsViewModel by viewModels()

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_shell)


        val navHost = supportFragmentManager.findFragmentById(R.id.navHost) as NavHostFragment
        findViewById<BottomNavigationView>(R.id.bottomNav)
            .setupWithNavController(navHost.navController)

        deviceHeader = findViewById(R.id.deviceHeader)
        liveSummary  = findViewById(R.id.liveSummary)

        deviceHeader.setOnDisconnectClick { BLEManager.disconnect() }

        lifecycleScope.launch {
            repeatOnLifecycle(Lifecycle.State.STARTED) {
                launch {
                    (application as App).repo.settings.collect { s ->
                        BLEManager.applyConfig(
                            autoReconnectEnabled = s.ar.autoReconnect,
                            companyId            = s.ar.companyId,
                            deviceId             = s.ar.deviceId6,
                            arTimeoutMs          = s.ar.timeoutMs,
                            arRetryMs            = s.ar.retryInterval,
                            scanMode             = s.ble.scanMode,
                            cleanupDurationMs    = s.ble.cleanupDurationMs,
                            filterScanDevice     = s.ble.filterScanDevice
                        )
                        viewModel.applyConfig(s.analy.maxPoints)
                    }
                }

                launch {
                    BLEManager.state.collect { state ->
                        updateUiForState(state)
                    }
                }
            }
        }
    }
    private fun updateUiForState(state: BleState){
        when(state){
            is BleState.Connected -> {
                val name = state.name ?: "Unknown"
                deviceHeader.setConnectionTitle("$name • ${getString(R.string.status_connected)}")
                deviceHeader.setSubtitle("")
                deviceHeader.setDisconnectVisible(true)

                val telem = state.telemetry
                if(telem != null){
                    liveSummary.isVisible = true
                    liveSummary.setRpm(telem.rpm)
                    liveSummary.setAngle(telem.angle)
                    liveSummary.setFlags(telem.rawFlags)
                    liveSummary.setState(telem.opState.displayName)
                }else{
                    liveSummary.isVisible = false
                }

            }
            is BleState.Connecting -> {
                deviceHeader.setConnectionTitle(getString(R.string.status_connecting))
                deviceHeader.setSubtitle("")
                deviceHeader.setDisconnectVisible(false)
                liveSummary.isVisible = false
            }
            is BleState.Scanning -> {
                deviceHeader.setConnectionTitle(getString(R.string.status_scanning)) // "Scanning..."
                deviceHeader.setSubtitle("")
                deviceHeader.setDisconnectVisible(false)
                liveSummary.isVisible = false
            }
            is BleState.Disconnected -> {
                deviceHeader.setConnectionTitle(getString(R.string.msg_not_connected))
                deviceHeader.setSubtitle("")
                deviceHeader.setDisconnectVisible(false)
                liveSummary.isVisible = false
            }
        }
    }
}
