package com.remotemotorcontroller.log

import com.remotemotorcontroller.ble.Telemetry
import java.util.Collections

object TelemetryLogger {
    private val sessionData = Collections.synchronizedList(mutableListOf<TelemetryRecord>())
    private var isLogging = false

    fun startLogging(){
        sessionData.clear()
        isLogging = true
    }

    fun stopLogging(){
        isLogging = false
    }

    fun log(telemetry: Telemetry){
        if(!isLogging)  return;

        val record = TelemetryRecord(
            System.currentTimeMillis(),
            telemetry.rpm,
            telemetry.angle,
            telemetry.opState.rawValue,
            telemetry.rawFlags
        )

        sessionData.add(record)
    }

    fun getCsvData(): String{
        val builder = java.lang.StringBuilder()

        builder.append("Timestamp(ms),RPM,Angle(deg),State,Flags\n")

        synchronized(sessionData){
            for(record in sessionData){
                builder.append("${record.timestampMs},${record.rpm},${record.angle},${record.state}," +
                        "${record.flag}\n")
            }
        }
        return builder.toString()
    }
}