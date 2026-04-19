package com.remotemotorcontroller.ui

import android.annotation.SuppressLint
import android.os.Bundle
import android.util.Log
import android.view.View
import android.widget.Button
import android.widget.Toast
import androidx.compose.material3.OutlinedButton
import androidx.core.widget.doAfterTextChanged
import androidx.fragment.app.Fragment
import com.google.android.material.button.MaterialButton
import com.google.android.material.button.MaterialButtonToggleGroup
import com.google.android.material.textfield.TextInputEditText
import com.remotemotorcontroller.R
import com.remotemotorcontroller.ble.BLEManager
import com.remotemotorcontroller.log.TelemetryLogger
import com.remotemotorcontroller.log.TelemetryRecord


class ControlFragment : Fragment(R.layout.fragment_control) {

    private lateinit var targetRpmEditText: TextInputEditText
    private lateinit var targetAngleEditText: TextInputEditText

    private lateinit var startStopMotorButton: MaterialButton
    private lateinit var sendAngleButton: MaterialButton

    private lateinit var calibrateButton: Button

    private lateinit var rpmToggleGroup: MaterialButtonToggleGroup

    private lateinit var angleToggleGroup: MaterialButtonToggleGroup

    private lateinit var kiEditText: TextInputEditText
    private lateinit var kpEditText: TextInputEditText
    private lateinit var imaxEditText: TextInputEditText
    private lateinit var updateConstraintsButton: Button



    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        targetRpmEditText = view.findViewById(R.id.inputRpm)
        targetAngleEditText = view.findViewById(R.id.inputAngle)

        startStopMotorButton = view.findViewById(R.id.buttonStartStopRpm)
        sendAngleButton = view.findViewById(R.id.buttonSendAngle)
        calibrateButton = view.findViewById(R.id.buttonCalibrate)

        rpmToggleGroup = view.findViewById(R.id.toggleRpmDirection)
        angleToggleGroup = view.findViewById(R.id.toggleAngleDirection)

        kiEditText = view.findViewById(R.id.inputKi)
        kpEditText = view.findViewById(R.id.inputKp)
        imaxEditText = view.findViewById(R.id.inputILimit)
        updateConstraintsButton = view.findViewById(R.id.buttonUpdateConstants)

        kpEditText.doAfterTextChanged {
            ControlUIState.kpInput = it.toString()
        }
        kiEditText.doAfterTextChanged {
            ControlUIState.kiInput = it.toString()
        }
        imaxEditText.doAfterTextChanged {
            ControlUIState.imaxInput = it.toString()
        }

        targetRpmEditText.doAfterTextChanged {
            ControlUIState.rpmInput = it.toString()
        }
        targetAngleEditText.doAfterTextChanged {
            ControlUIState.angleInput = it.toString()
        }

        restoreUiState()

        rpmToggleGroup.addOnButtonCheckedListener{ _, checkedId, isChecked ->
            if (isChecked) ControlUIState.isRpmCcw = (checkedId == R.id.btnRpmCcw)
        }
        angleToggleGroup.addOnButtonCheckedListener { _, checkedId, isChecked ->
            if (isChecked) ControlUIState.isAngleCcw = (checkedId == R.id.btnAngleCcw)
        }

        calibrateButton.setOnClickListener {
            BLEManager.activeSession?.calibrate()
            Toast.makeText(requireContext(), "Calibrating...", Toast.LENGTH_SHORT).show()
        }

        startStopMotorButton.setOnClickListener {
            if (!ControlUIState.isMotorRunning) { // MOTOR STARTING
                TelemetryLogger.startLogging()

                var rpm = targetRpmEditText.text.toString().toIntOrNull()
                if (rpm != null) {
                    // COUNTER CLOCKWISE == NEGATIVE VALUES FOR MOTOR
                    if(rpmToggleGroup.checkedButtonId == R.id.btnRpmCcw){
                        rpm = -rpm
                    }

                    ControlUIState.isMotorRunning = true
                    setStopUi()
                    BLEManager.activeSession?.setSpeed(rpm)
                    Toast.makeText(
                        requireContext(),
                        "Starting motor at $rpm RPM",
                        Toast.LENGTH_SHORT
                    ).show()
                } else {
                    targetRpmEditText.error = "Invalid Number"
                }
            } else { // MOTOR STOPPING
                TelemetryLogger.stopLogging()
                ControlUIState.isMotorRunning = false
                setStartUi()

                BLEManager.activeSession?.shutdown()
                Toast.makeText(requireContext(), "Stopping motor", Toast.LENGTH_SHORT).show()
            }
        }
        sendAngleButton.setOnClickListener {
            var angle = targetAngleEditText.text.toString().toIntOrNull()
            if (angle != null) {
                if(angleToggleGroup.checkedButtonId == R.id.btnAngleCcw){
                    angle = -angle
                }
                BLEManager.activeSession?.setPosition(angle)
                Toast.makeText(requireContext(), "Moving to $angle degrees", Toast.LENGTH_SHORT).show()
            } else {
                targetAngleEditText.error = "Invalid Number"
            }
        }

        updateConstraintsButton.setOnClickListener {
            var kp = kpEditText.text.toString().toFloatOrNull()
            var ki = kiEditText.text.toString().toFloatOrNull()
            var iMax = imaxEditText.text.toString().toFloatOrNull()

            if(kp == null){
                kpEditText.setText("0.3")
                kp = 0.3F
            }
            if(ki == null){
                kiEditText.setText("0.04")
                ki = 0.04F
            }
            if(iMax == null) {
                imaxEditText.setText("2000")
                iMax = 2000F
            }
            BLEManager.activeSession?.sendPidTuning(kp,ki,iMax)
        }
    }

    private fun restoreUiState(){
        targetRpmEditText.setText(ControlUIState.rpmInput)
        targetAngleEditText.setText(ControlUIState.angleInput)
        kiEditText.setText(ControlUIState.kiInput)
        kpEditText.setText(ControlUIState.kpInput)
        imaxEditText.setText(ControlUIState.imaxInput)

        if(ControlUIState.isRpmCcw){
            rpmToggleGroup.check(R.id.btnRpmCcw)
        }
        else{
            rpmToggleGroup.check(R.id.btnRpmCw)
        }

        if(ControlUIState.isAngleCcw){
            angleToggleGroup.check(R.id.btnAngleCcw)
        }
        else{
            angleToggleGroup.check(R.id.btnAngleCw)
        }

        if(ControlUIState.isMotorRunning){
            setStopUi()
        } else {
            setStartUi()
        }
    }

    private fun setStartUi() {
        startStopMotorButton.text = "START MOTOR"
        startStopMotorButton.setIconResource(R.drawable.ic_play)

        targetRpmEditText.isEnabled = true
        rpmToggleGroup.isEnabled = true
    }

    private fun setStopUi() {
        startStopMotorButton.text = "STOP MOTOR"
        startStopMotorButton.setIconResource(R.drawable.ic_stop)

        targetRpmEditText.isEnabled = false
    }
}