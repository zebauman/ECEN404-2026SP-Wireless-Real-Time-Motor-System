package com.remotemotorcontroller.ui

import android.net.Uri
import android.os.Bundle
import android.view.View
import android.widget.ArrayAdapter
import android.widget.Spinner
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.fragment.app.Fragment
import androidx.lifecycle.lifecycleScope
import com.google.android.material.button.MaterialButton
import com.google.android.material.materialswitch.MaterialSwitch
import com.google.android.material.textfield.TextInputEditText
import com.remotemotorcontroller.settings.SettingsRepository
import com.remotemotorcontroller.R
import com.remotemotorcontroller.log.TelemetryLogger
import com.remotemotorcontroller.utils.hexToBytesOrNull
import com.remotemotorcontroller.utils.toHex
import kotlinx.coroutines.flow.first
import kotlinx.coroutines.launch


class SettingsFragment : Fragment(R.layout.fragment_settings) {

    private lateinit var repo: SettingsRepository

    private val exportCsvLauncher = registerForActivityResult(ActivityResultContracts.CreateDocument("text/csv")){
        uri ->
            uri?.let { saveDataToFile(it)}
    }

    override fun onViewCreated(view:View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        repo = SettingsRepository(requireContext())

        view.findViewById<View>(R.id.rowBleConfig).setOnClickListener{ showBleSheet() }
        view.findViewById<View>(R.id.rowARConfig).setOnClickListener { showArSheet() }
        view.findViewById<View>(R.id.rowAnalyConfig).setOnClickListener { showAnalySheet() }

        view.findViewById<View>(R.id.rowExport).setOnClickListener {
            TelemetryLogger.stopLogging()

            val fileName = "MotorTelemetry_${System.currentTimeMillis()}.csv"
            exportCsvLauncher.launch(fileName)
        }
    }

    private fun saveDataToFile(uri: Uri){
        try{
            requireContext().contentResolver.openOutputStream(uri)?.use { outputStream ->
                val csvData = TelemetryLogger.getCsvData()

                outputStream.write(csvData.toByteArray())

                Toast.makeText(requireContext(), "Data exported successfully!", Toast.LENGTH_SHORT).show()

                TelemetryLogger.startLogging()
            }
        } catch (e: Exception) {
            e.printStackTrace()
            Toast.makeText(requireContext(), "Failed to export data", Toast.LENGTH_SHORT).show()
        }
    }

    fun showBleSheet(){
        val sheet = ConfigBottomSheet.new("BLE Configuration", R.layout.bs_ble_config)
        sheet.show(parentFragmentManager,"ble")

        parentFragmentManager.executePendingTransactions()
        val root = sheet.dialog?.findViewById<View>(R.id.bsContent) ?: return

        val swFilter = root.findViewById<MaterialSwitch>(R.id.swFilter)
        val spMode = root.findViewById<Spinner>(R.id.spScanMode)
        val inCleanup = root.findViewById<TextInputEditText>(R.id.inCleanup)
        val saveButton = root.findViewById<MaterialButton>(R.id.btnSaveBle)

        ArrayAdapter.createFromResource(
            requireContext(), R.array.scan_mode_names, android.R.layout.simple_spinner_item
        ).also {
            it.setDropDownViewResource(android.R.layout.simple_spinner_dropdown_item)
            spMode.adapter = it
        }

        // DISPLAY THE CURRENT VALUES
        lifecycleScope.launch{
            val cfg = repo.settings.first()
            swFilter.isChecked = cfg.ble.filterScanDevice

            val values = resources.getIntArray(R.array.scan_mode_values)
            val idx = values.indexOf(cfg.ble.scanMode).coerceAtLeast(0)
            spMode.setSelection(idx)

            inCleanup.setText(cfg.ble.cleanupDurationMs.toString())
        }

        saveButton.setOnClickListener {
            lifecycleScope.launch{
                val values = resources.getIntArray(R.array.scan_mode_values)
                val newMode = values.getOrNull(spMode.selectedItemPosition) ?: 2

                repo.enableFilter(swFilter.isChecked)
                repo.setScanMode(newMode)
                repo.setCleanupDuration(inCleanup.text.toString().toLongOrNull() ?: 5_000L)

                Toast.makeText(requireContext(), "Saved", Toast.LENGTH_SHORT).show()
                sheet.dismiss()
            }
        }
    }

    fun showArSheet(){
        val sheet = ConfigBottomSheet.new("Auto-Reconnect Configuration", R.layout.bs_ar_config)
        sheet.show(parentFragmentManager,"ar")

        parentFragmentManager.executePendingTransactions()
        val root = sheet.dialog?.findViewById<View>(R.id.bsContent) ?: return

        val swAutoReconnect = root.findViewById<MaterialSwitch>(R.id.swAutoReconnect)
        val inCompanyIdHex = root.findViewById<TextInputEditText>(R.id.inCompanyIdHex)
        val inDeviceIdHex = root.findViewById<TextInputEditText>(R.id.inDeviceIdHex)
        val inTimeout = root.findViewById<TextInputEditText>(R.id.inTimeout)
        val inRetry = root.findViewById<TextInputEditText>(R.id.inRetry)
        val btnSaveAr = root.findViewById<MaterialButton>(R.id.btnSaveAr)

        lifecycleScope.launch {
            val cfg = repo.settings.first()

            swAutoReconnect.isChecked = cfg.ar.autoReconnect
            inCompanyIdHex.setText(Integer.toHexString(cfg.ar.companyId))
            inDeviceIdHex.setText(cfg.ar.deviceId6?.toHex() ?: "")
            inTimeout.setText(cfg.ar.timeoutMs.toString())
            inRetry.setText(cfg.ar.retryInterval.toString())
        }
        btnSaveAr.setOnClickListener {
            lifecycleScope.launch {
                repo.enableAutoReconnect(swAutoReconnect.isChecked)

                val cidTxt = inCompanyIdHex.text.toString().trim().ifEmpty { "706d" }
                val cid = runCatching { cidTxt.toInt(16) }.getOrNull()
                if (cid == null) { inCompanyIdHex.error = "Invalid hex (e.g., 706d)"; return@launch }
                inCompanyIdHex.error = null
                repo.setCompanyId(cid)

                val didTxt = inDeviceIdHex.text.toString().trim()
                if (didTxt.isEmpty()) {
                    repo.clearDeviceId6()
                } else {
                    val bytes = didTxt.hexToBytesOrNull()
                    if (bytes == null || bytes.size != 6) { inCompanyIdHex.error = "12 hex chars required"; return@launch }
                    inCompanyIdHex.error = null
                    repo.setDeviceId6(bytes)
                }

                val t  = inTimeout.text.toString().toLongOrNull()
                val rt = inRetry.text.toString().toLongOrNull()
                if (t == null || rt == null) {
                    if (t == null) inTimeout.error = "Enter ms"
                    if (rt == null) inRetry.error = "Enter ms"
                    return@launch
                }
                inTimeout.error = null; inRetry.error = null
                repo.setAutoReconnectTimes(t, rt)

                Toast.makeText(requireContext(), "Saved", android.widget.Toast.LENGTH_SHORT).show()
                sheet.dismiss()
            }
        }

        }
    fun showAnalySheet(){
        val sheet = ConfigBottomSheet.new("Analytic Settings", R.layout.bs_analy_config)
        sheet.show(parentFragmentManager,"ar")

        parentFragmentManager.executePendingTransactions()
        val root = sheet.dialog?.findViewById<View>(R.id.bsContent) ?: return

        val inMaxPts = root.findViewById<TextInputEditText>(R.id.inMaxPts)
        val btnSaveAnaly = root.findViewById<MaterialButton>(R.id.btnSaveAnaly)

        lifecycleScope.launch {
            val cfg = repo.settings.first()
            inMaxPts.setText(cfg.analy.maxPoints.toString())
        }
        btnSaveAnaly.setOnClickListener {
            lifecycleScope.launch{
                val newMaxPts = inMaxPts.text.toString().toIntOrNull() ?: 600
                if(newMaxPts <= 0){
                    inMaxPts.error = "Enter Points Greater than 0"
                    return@launch
                }
                repo.setMaxPoints(newMaxPts)
                Toast.makeText(requireContext(), "Saved", android.widget.Toast.LENGTH_SHORT).show()
                sheet.dismiss()

            }
        }

    }
}