package org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm

import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients

class ShoulderConfig(@JvmField var pid : CustomPIDFCoefficients = CustomPIDFCoefficients(0.02,0.0,0.0027,0.0),
                     @JvmField var angles : ShoulderAngles = ShoulderAngles(),
                     @JvmField var maxPower : Double = 1.0,
                     @JvmField var Kf : Double = 0.1,
) { }
class ShoulderAngles(@JvmField var min: Double = -35.0,
                     @JvmField var max: Double = 110.0,
                     @JvmField var start : Double = -37.0,
                     @JvmField var travel : Double = -25.0,
                     @JvmField var beltInstallation : Double = 0.0,
                     @JvmField var homingAngleIncrement : Double = 1.0,

                     @JvmField var samplePreIntake : Double = -15.0,
                     @JvmField var sampleIntake : Double = -5.0,
                     @JvmField var samplePrePositionIntake : Double = -15.0,
                     @JvmField var sampleScoreHigh : Double = 105.0,
                     @JvmField var sampleScoreLow : Double = 40.0,

                     @JvmField var specimenIntakeWall : Double = -30.0,
                     @JvmField var specimenIntakeWallLifted : Double = -20.0,
                     @JvmField var specimenAboveHigh : Double = 35.0,
                     @JvmField var specimenAboveLow : Double = 15.0,
                     @JvmField var specimenScoreHigh : Double = 0.0,
                     @JvmField var specimenScoreLow : Double = -20.0,

                     @JvmField var parkObservation : Double = 20.0,
                     @JvmField var parkTouchBar : Double = 20.0

) { }