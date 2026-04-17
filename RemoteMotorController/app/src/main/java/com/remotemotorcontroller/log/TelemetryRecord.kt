package com.remotemotorcontroller.log

data class TelemetryRecord(
    val timestampMs: Long,
    val rpm: Int,
    val angle: Int,
    val state: Int,
    val flag: Int
)   // LET ALL THE PARAMETERS FOR DATA RECORD BE THE RAW VALUES (ONLY THE INT REPRESENTATION) TO SAVE SPACE

