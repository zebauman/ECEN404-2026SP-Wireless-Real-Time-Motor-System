package com.remotemotorcontroller.ui.widgets

import android.content.Context
import android.graphics.Color
import android.util.AttributeSet
import android.view.LayoutInflater
import android.widget.FrameLayout
import android.widget.TextView
import androidx.core.flagging.Flags
import com.google.android.material.card.MaterialCardView
import com.google.android.material.chip.Chip
import com.remotemotorcontroller.R
import androidx.core.graphics.toColorInt
import com.remotemotorcontroller.ble.BLEContract

class LiveSummaryView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : FrameLayout(context, attrs, defStyleAttr) {

    private val textRpm: TextView
    private val textAngle: TextView
    private val textState: TextView
    private val textFlags: TextView

    init {
        LayoutInflater.from(context).inflate(R.layout.include_live_summary, this, true)

        textRpm = findViewById(R.id.textRpm)
        textAngle = findViewById(R.id.textAngle)
        textState = findViewById(R.id.textState)
        textFlags = findViewById(R.id.textFlags)
    }

    fun setRpm(rpm: Int) {
        textRpm.text = context.getString(R.string.label_rpm, rpm)
    }

    fun setAngle(angle: Int) {
        textAngle.text = context.getString(R.string.label_angle, angle)
    }

    fun setState(state: String){
        textState.text = state
    }

    fun setFlags(activeFlags: Int){

        if(activeFlags == 0){
            textFlags.text = "None"
            textFlags.setTextColor("#4CAF50".toColorInt())
            return
        }

        val warnings = mutableListOf<String>()

        if((activeFlags and BLEContract.MotorFlag.SYNC_BAD) != 0){
            warnings.add("SYNC LOST")
        }
        if((activeFlags and BLEContract.MotorFlag.OVERHEAT) != 0){
            warnings.add("OVERHEAT")
        }
        if((activeFlags and BLEContract.MotorFlag.STALL) != 0){
            warnings.add("STALL")
        }

        textFlags.text = warnings.joinToString(" • ")
        textFlags.setTextColor(Color.RED)
    }
}
