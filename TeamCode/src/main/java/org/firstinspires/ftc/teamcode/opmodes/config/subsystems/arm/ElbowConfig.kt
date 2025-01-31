package org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm

import org.firstinspires.ftc.teamcode.opmodes.teleop.SlewRateLimits

class ElbowConfig(@JvmField var angles : ElbowAngles = ElbowAngles(),
                  @JvmField var angleOffset : Double = -20.0,
                  @JvmField var Kf : Double = 0.15,
                  @JvmField var angleSlewLimit: SlewRateLimits = SlewRateLimits(240.0, 240.0),
                  @JvmField var targetTolerance : Double = 3.0, // degrees
) {}

class ElbowAngles(   @JvmField var start : Double = 135.0,
                     @JvmField var travel : Double = 90.0,
                     @JvmField var beltInstallation : Double = 90.0,

                     @JvmField var samplePreIntake : Double = 0.0,
                     @JvmField var sampleIntake : Double= -32.0,
                     @JvmField var samplePrePositionIntake : Double = -15.0,
                     @JvmField var sampleScoreHigh : Double = 115.0,
                     @JvmField var sampleScoreLow : Double = 150.0,

                     @JvmField var specimenIntakeWall : Double = 90.0,
                     @JvmField var specimenIntakeWallLifted : Double = 90.0,
                     @JvmField var specimenAboveHigh : Double = 145.0,
                     @JvmField var specimenAboveLow : Double = 90.0,
                     @JvmField var specimenScoreHigh : Double = 90.0,
                     @JvmField var specimenScoreLow : Double = 90.0,

                     @JvmField var parkObservation : Double = 90.0,
                     @JvmField var parkTouchBar : Double = 75.0
) {}