package com.remotemotorcontroller.ble

sealed class BleState{
    object Disconnected : BleState()
    object Scanning : BleState()

    data class Connecting (val name: String?) : BleState()

    data class Connected(
        val name: String?,
        val telemetry: Telemetry? = null  // CONTAINS the live RPM/Angle data
    ) : BleState()
}

data class Telemetry(
    val opState: MotorOpState,
    val rpm: Int,
    val angle: Int,
    val rawFlags: Int
){
    companion object{   // USING COMPANION OBJECT for INIT TO BE ABLE TO RETURN NULL IF APPLICABLE
        fun fromBytes(value: ByteArray) : Telemetry? {
            if(value.size < 9) return null
            val statusByte = value[0]

            val opState = MotorOpState.fromByte(statusByte)
            val flags = statusByte.toInt() and BLEContract.MOTOR_FLAG_MASK

            val speed = ((value[1].toInt() and 0xFF) or ((value[2].toInt() and 0xFF) shl 8) or
                    ((value[3].toInt() and 0xFF) shl 16) or ((value[4].toInt() and 0xFF) shl 24))
            val position = ((value[5].toInt() and 0xFF) or ((value[6].toInt() and 0xFF) shl 8) or
                    (((value[7]).toInt() and 0xFF) shl 16) or ((value[8].toInt() and 0xFF) shl 24))
            return Telemetry(opState, speed, position,flags)
        }
    }
}
