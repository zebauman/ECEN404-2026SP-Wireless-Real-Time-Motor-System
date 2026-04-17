package com.remotemotorcontroller.ble

enum class MotorOpState(val rawValue: Int, val displayName: String) {
    STOPPED(BLEContract.MotorState.STOPPED, "STOPPED"),
    RUNNING_SPEED(BLEContract.MotorState.RUNNING_SPEED, "SPEED MODE"),
    RUNNING_POS(BLEContract.MotorState.RUNNING_POS, "POSITION MODE"),
    ESTOP(BLEContract.MotorState.ESTOP, "E-STOP ACTIVATED"),
    RESTART(BLEContract.MotorState.RESTART, "RESTARTING"),
    FAULT(BLEContract.MotorState.FAULT, "HARDWARE FAULT"),
    UNKNOWN(-1, "UNKNOWN");

    companion object{
        fun fromByte(value: Byte): MotorOpState {
            val isolatedState = (value.toInt() and BLEContract.MOTOR_STATE_MASK)
            return entries.find { it.rawValue == isolatedState } ?: UNKNOWN
        }
    }
}