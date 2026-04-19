package com.remotemotorcontroller.ble

import android.bluetooth.BluetoothGatt
import android.bluetooth.BluetoothGattCharacteristic

class BleMotorSession(
    gatt: BluetoothGatt,
    private val requestQueue: BleRequestQueue
    ) {
    private val charCmd = gatt.getService(BLEContract.SERVICE_MOTOR)?.getCharacteristic(BLEContract.CHAR_CMD)
    private val charHeartbeat = gatt.getService(BLEContract.SERVICE_MOTOR)?.getCharacteristic(
        BLEContract.CHAR_HEARTBEAT)

    fun sendCommand(cmd: Byte,
                    value: Int,
                    writeType: Int = BluetoothGattCharacteristic.WRITE_TYPE_DEFAULT,
                    priority: Int = BleRequestQueue.PRIORITY_LOW){
        val payload = byteArrayOf(cmd,
            (value and 0xFF).toByte(),
            ((value shr 8) and 0xFF).toByte(),
            ((value shr 16) and 0xFF).toByte(),
            ((value shr 24) and 0xFF).toByte()
        )
        charCmd?.let { requestQueue.enqueueWrite(
            it,
            payload,
            writeType,
            priority
        )

        }
    }

    fun sendPidTuning(kp: Float, ki: Float, iLimit: Float){
        val kpBits = kp.toRawBits()
        val kiBits = ki.toRawBits()
        val lBits = iLimit.toRawBits()

        sendCommand(BLEContract.CMD_SET_KP, kpBits)
        sendCommand(BLEContract.CMD_SET_KI, kiBits)
        sendCommand(BLEContract.CMD_SET_ILIMIT, lBits)
    }

    fun setSpeed(rpm: Int) = sendCommand(BLEContract.CMD_SPEED, rpm)
    fun setPosition(pos: Int) = sendCommand(BLEContract.CMD_POSITION, pos)
    fun calibrate() = sendCommand(BLEContract.CMD_CALIBRATE, 0)
    fun shutdown() = sendCommand(BLEContract.CMD_SHUTDOWN, 0, BleRequestQueue.PRIORITY_CRITICAL)

    fun sendHeartBeat(count: Int){

        charHeartbeat?.let{ requestQueue.enqueueWrite(
            it,
            byteArrayOf(count.toByte()),
            BluetoothGattCharacteristic.WRITE_TYPE_NO_RESPONSE,
            BleRequestQueue.PRIORITY_HIGH
        )
        }
    }
}