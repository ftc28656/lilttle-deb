package org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake

class IntakeConfig(@JvmField var speeds: IntakeServoSpeeds = IntakeServoSpeeds(),
                   @JvmField var colorLimits : IntakeColorLimits = IntakeColorLimits(),
                   @JvmField var distanceThreshold: Double = 3.0,
                   @JvmField var crTimeout: Double = 3.0,
                   @JvmField var crPause: Double = 0.050,
                   @JvmField var autoStopDelay: Double = 1.0,
    ) { }
class IntakeServoSpeeds(@JvmField var intake : Double = 1.0,
                        @JvmField var outtake : Double = 0.0,
                        @JvmField var stop : Double = 0.5,
                        @JvmField var hold : Double = 0.5
) { }

class IntakeColorLimits(@JvmField var red : IntakeHueLimits = IntakeHueLimits(0.0f, 40.0f),
                        @JvmField var yellow : IntakeHueLimits = IntakeHueLimits(70.0f, 140.0f),
                        @JvmField var blue : IntakeHueLimits = IntakeHueLimits(180.0f, 250.0f)) {

}
class IntakeHueLimits(@JvmField var minHue : Float,
                      @JvmField var maxHue : Float)