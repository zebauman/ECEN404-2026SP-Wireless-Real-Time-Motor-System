package com.remotemotorcontroller.ui

object ControlUIState {
    var rpmInput: String = ""
    var angleInput: String = ""

    var kpInput: String = ""
    var kiInput: String = ""
    var imaxInput: String = ""

    // CLOCKWISE IS DEFAULT
    var isRpmCcw: Boolean = false
    var isAngleCcw: Boolean = false

    var isMotorRunning = false
}