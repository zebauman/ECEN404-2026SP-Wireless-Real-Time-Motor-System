package com.remotemotorcontroller.ble

import java.util.UUID

object BLEContract {
    val SERVICE_MOTOR: UUID = UUID.fromString("c52081ba-e90f-40e4-a99f-ccaa4fd11c15")
    val CHAR_CMD: UUID = UUID.fromString("d10b46cd-412a-4d15-a7bb-092a329eed46")

    val CHAR_HEARTBEAT: UUID = UUID.fromString("2215d558-c569-4bd1-8947-b4fd5f9432a0")
    val CHAR_TELEM: UUID = UUID.fromString("17da15e5-05b1-42df-8d9d-d7645d6d9293")

    val DESC_CCCD: UUID = UUID.fromString("00002902-0000-1000-8000-00805f9b34fb")

    const val CMD_SHUTDOWN:  Byte = 0x00
    const val CMD_CALIBRATE: Byte = 0x01
    const val CMD_SPEED:     Byte = 0x02
    const val CMD_POSITION:  Byte = 0x03
    const val CMD_SET_KP:    Byte = 0x04
    const val CMD_SET_KI:    Byte = 0x05
    const val CMD_SET_ILIMIT:Byte = 0x06

    const val MOTOR_STATE_MASK = 0x0F
    const val MOTOR_FLAG_MASK  = 0xF0

    object MotorState {
        const val STOPPED       = 0x00

        const val RUNNING_SPEED = 0x01
        const val RUNNING_POS   = 0x02
        const val ESTOP         = 0x03
        const val RESTART       = 0x04
        const val FAULT         = 0x05
    }

    // Flags are Ints
    object MotorFlag {
        const val SYNC_BAD = 0x10
        const val OVERHEAT = 0x20
        const val STALL    = 0x40
    }
}