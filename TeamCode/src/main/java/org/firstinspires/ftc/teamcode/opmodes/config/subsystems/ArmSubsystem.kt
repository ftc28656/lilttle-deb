package org.firstinspires.ftc.teamcode.opmodes.config.subsystems

import android.graphics.Color
import com.qualcomm.robotcore.hardware.HardwareMap
import org.firstinspires.ftc.teamcode.opmodes.config.RobotConstants
import kotlin.math.abs

class ArmSubsystem(hardwareMap: HardwareMap) {
    private val intakeServo = hardwareMap.servo.get("intakeServo")
    private val elbowServo = hardwareMap.servo.get("elbowServo")
    private val shoulderServo1 = hardwareMap.servo.get("shoulderServo1")
    private val shoulderServo2 = hardwareMap.servo.get("shoulderServo2")
    private val shoulderServo3 = hardwareMap.servo.get("shoulderServo3")
    private val shoulderServos = arrayOf(shoulderServo1, shoulderServo2, shoulderServo3)
    private val colorSensor = hardwareMap.colorSensor.get("colorSensor")

    // Analog inputs for Axon 4th wires
    private val elbowServoAnalogInput = hardwareMap.analogInput.get("elbowServoAnalogInput")
    private val shoulderServo1AnalogInput = hardwareMap.analogInput.get("shoulderServo1AnalogInput")

    var targetPosition: ArmPositions = ArmPositions.START
        set(value) {
            field = value
            setArmPosition(value)
        }
    var intakeState: IntakeStates = IntakeStates.STOPPED
        set(value) {
            field = value
            setIntakeState()
        }
    val intakeElement: IntakeElement
        get() {
            // TODO: this is wrong, but it's just a placeholder.  Need to convert to HSV and then compare Hue values.  Need to test this.
            val color = getHSVColor()
            return when(color.hue) {
                in 0.0..60.0 -> IntakeElement.RED
                in 60.0..180.0 -> IntakeElement.BLUE
                in 180.0..300.0 -> IntakeElement.YELLOW
                else -> IntakeElement.NONE
            }
        }
    val shoulderAngle: Double
        get() {
            val shoulderVoltage = shoulderServo1AnalogInput.voltage
            val conversionFactor = 180.0 / 3.3 // TODO: this is a placeholder, need to determine this. Depends on how servo is mounted.
            return shoulderVoltage * conversionFactor
        }
    val elbowAngle: Double
        get() {
            val elbowVoltage = elbowServoAnalogInput.voltage
            val conversionFactor = 180.0 / 3.3 // TODO: this is a placeholder, need to determine this. Depends on how servo is mounted.
            return elbowVoltage * conversionFactor
        }

    fun init() {
        // Initialize the color sensor LED (just leave it on?)
        colorSensor.enableLed(true)
    }

    fun update() {
        // not sure if this is necessary, but it's here for now.  Called each loop.  Useful if we have a PID or something.
    }

    private fun setIntakeState() {
        when(intakeState) {
            IntakeStates.INTAKING -> {
                intakeServo.position = RobotConstants.intakeServoSpeed
            }
            IntakeStates.OUTTAKING -> {
                intakeServo.position = RobotConstants.outtakeServoSpeed
            }
            IntakeStates.STOPPED -> {
                intakeServo.position = RobotConstants.stopIntakeServoSpeed
            }
        }
    }
    private fun getHSVColor(): HSV {
        val hsvValues = floatArrayOf(0f, 0f, 0f)
        Color.RGBToHSV(colorSensor.red(), colorSensor.green(), colorSensor.blue(), hsvValues)
        return HSV(hsvValues[0], hsvValues[1], hsvValues[2])
    }
    private fun setArmPosition(position: ArmPositions) {
        when (position) {
            ArmPositions.PARK -> {
                elbowServo.position = RobotConstants.elbowParkAngle.toElbowPosition()
                shoulderServos.forEach { it.position = RobotConstants.shoulderParkAngle.toShoulderPosition() }
            }
            ArmPositions.SAMPLE_INTAKE -> {
                elbowServo.position = RobotConstants.elbowSampleIntakeAngle.toElbowPosition()
                shoulderServos.forEach { it.position = RobotConstants.shoulderSampleIntakeAngle.toShoulderPosition() }
            }
            ArmPositions.SPECIMEN_INTAKE -> {
                elbowServo.position = RobotConstants.elbowSpecimenIntakeAngle.toElbowPosition()
                shoulderServos.forEach { it.position = RobotConstants.shoulderSpecimenIntakeAngle.toShoulderPosition() }
            }
            ArmPositions.SCORE_BUCKET_HIGH -> {
                elbowServo.position = RobotConstants.elbowScoreBucketHighAngle.toElbowPosition()
                shoulderServos.forEach { it.position = RobotConstants.shoulderScoreBucketHighAngle.toShoulderPosition() }
            }
            ArmPositions.SCORE_BUCKET_MEDIUM -> {
                elbowServo.position = RobotConstants.elbowScoreBucketMediumAngle.toElbowPosition()
                shoulderServos.forEach { it.position = RobotConstants.shoulderScoreBucketMediumAngle.toShoulderPosition() }
            }
            ArmPositions.SCORE_BUCKET_LOW -> {
                elbowServo.position = RobotConstants.elbowScoreBucketLowAngle.toElbowPosition()
                shoulderServos.forEach { it.position = RobotConstants.shoulderScoreBucketLowAngle.toShoulderPosition() }
            }
            ArmPositions.SCORE_CHAMBER_LOW -> {
                elbowServo.position = RobotConstants.elbowScoreChamberLowAngle.toElbowPosition()
                shoulderServos.forEach { it.position = RobotConstants.shoulderScoreChamberLowAngle.toShoulderPosition() }
            }
            ArmPositions.SCORE_CHAMBER_HIGH -> {
                elbowServo.position = RobotConstants.elbowScoreChamberHighAngle.toElbowPosition()
                shoulderServos.forEach { it.position = RobotConstants.shoulderScoreChamberHighAngle.toShoulderPosition() }
            }
            ArmPositions.START -> {
                elbowServo.position = RobotConstants.elbowStartAngle.toElbowPosition()
                shoulderServos.forEach { it.position = RobotConstants.shoulderStartAngle.toShoulderPosition() }
            }
        }
    }
    private fun Double.toElbowPosition(): Double {
        // TODO: this is a placeholder, need to test this.  Need to convert from degrees to servo position [0.0 1.0]
        return this / 180.0
    }
    private fun Double.toShoulderPosition(): Double {
        // TODO: this is a placeholder, need to test this.  Need to convert from degrees to servo position [0.0 1.0]
        return this / 180.0
    }
    // Tolerance is in degrees defaulting to 2 degrees
    fun isArmAtTargetPosition(tolerance : Double = 2.0): Boolean {
        return when(targetPosition) {
            ArmPositions.PARK ->
                abs(elbowAngle - RobotConstants.elbowParkAngle) < tolerance &&
                        abs(shoulderAngle - RobotConstants.shoulderParkAngle) < tolerance
            ArmPositions.SAMPLE_INTAKE -> {
                abs(elbowAngle - RobotConstants.elbowSampleIntakeAngle) < tolerance &&
                        abs(shoulderAngle - RobotConstants.shoulderSampleIntakeAngle) < tolerance
            }
            ArmPositions.SPECIMEN_INTAKE -> {
                abs(elbowAngle - RobotConstants.elbowSpecimenIntakeAngle) < tolerance &&
                        abs(shoulderAngle - RobotConstants.shoulderSpecimenIntakeAngle) < tolerance
            }
            ArmPositions.SCORE_BUCKET_HIGH -> {
                abs(elbowAngle - RobotConstants.elbowScoreBucketHighAngle) < tolerance &&
                        abs(shoulderAngle - RobotConstants.shoulderScoreBucketHighAngle) < tolerance
            }
            ArmPositions.SCORE_BUCKET_MEDIUM -> {
                abs(elbowAngle - RobotConstants.elbowScoreBucketMediumAngle) < tolerance &&
                        abs(shoulderAngle - RobotConstants.shoulderScoreBucketMediumAngle) < tolerance
            }
            ArmPositions.SCORE_BUCKET_LOW -> {
                abs(elbowAngle - RobotConstants.elbowScoreBucketLowAngle) < tolerance &&
                        abs(shoulderAngle - RobotConstants.shoulderScoreBucketLowAngle) < tolerance
            }
            ArmPositions.SCORE_CHAMBER_LOW -> {
                abs(elbowAngle - RobotConstants.elbowScoreChamberLowAngle) < tolerance &&
                        abs(shoulderAngle - RobotConstants.shoulderScoreChamberLowAngle) < tolerance
            }
            ArmPositions.SCORE_CHAMBER_HIGH -> {
                abs(elbowAngle - RobotConstants.elbowScoreChamberHighAngle) < tolerance &&
                        abs(shoulderAngle - RobotConstants.shoulderScoreChamberHighAngle) < tolerance
            }
            ArmPositions.START -> {
                abs(elbowAngle - RobotConstants.elbowStartAngle) < tolerance &&
                        abs(shoulderAngle - RobotConstants.shoulderStartAngle) < tolerance
            }
        }
    }
}