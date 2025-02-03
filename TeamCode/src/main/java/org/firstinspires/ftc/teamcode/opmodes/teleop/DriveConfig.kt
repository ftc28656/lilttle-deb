package org.firstinspires.ftc.teamcode.opmodes.teleop

class DriveConfig(
    @JvmField var translationSlew: SlewRateLimits = SlewRateLimits(10.0, 100.0),
    @JvmField var rotationalSlew: SlewRateLimits = SlewRateLimits(10.0, 100.0),
    @JvmField var alignmentHeading: Double = 270.0, // to the drivers right
    ) { }

class SlewRateLimits(
    @JvmField var accel: Double = 1.0, // max power change per second for positive changes
    @JvmField var decel: Double = 100.0, // max power change per second for negative changes

){ }