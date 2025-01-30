package org.firstinspires.ftc.teamcode.opmodes.teleop

class DriveConfig(
    @JvmField var xMaxSlewRate: Double = 100.0, // max power change per second
    @JvmField var yMaxSlewRate: Double = 100.0, // max power change per second
    @JvmField var turnMaxSlewRate: Double = 1.0, // max power change per second
    @JvmField var alignmentHeading: Double = 270.0, // to the drivers right
    ) { }