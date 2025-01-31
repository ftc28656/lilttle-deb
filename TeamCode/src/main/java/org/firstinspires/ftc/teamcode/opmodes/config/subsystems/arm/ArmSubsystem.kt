package org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm

import android.util.Log
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.HardwareMap
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import edu.ncssm.ftc.electricmayhem.core.util.epsilonEquals
import org.firstinspires.ftc.teamcode.opmodes.config.LittleDebbie
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions.clamp
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer
import org.firstinspires.ftc.teamcode.util.SlewRateFilter
import org.firstinspires.ftc.teamcode.util.motion.MotionProfileGenerator
import org.firstinspires.ftc.teamcode.util.motion.MotionState
import kotlin.math.abs
import kotlin.math.cos

class ArmSubsystem(val hardwareMap: HardwareMap)  {

    // elbow
    lateinit var rightElbowServo: ServoImplEx
    lateinit var leftElbowServo: ServoImplEx
    lateinit var leftElbowPosition: AnalogInput
    lateinit var rightElbowPosition: AnalogInput

    // shoulder
    lateinit var shoulderMotor : DcMotorSimple
    lateinit var shoulderEncoderMotor : DcMotorEx
    private val shoulderPID = PIDFController(LittleDebbie.shoulder.pid)

    // mag limit switch
    lateinit var magLimit: DigitalChannel
    var state: ArmStates = ArmStates.START
        set(value) {
            previousState = field
            field = value
            updateTargetsForArmState(field)
        }
    var previousState = state
        private set;
    var targetShoulderAngle = LittleDebbie.shoulder.angles.start
        private set(value) {
            if(!field.epsilonEquals(value))
               atTargetTimer.resetTimer()

            when(state) {
                ArmStates.HOMING -> {
                    // we have to remove the min/max as we don't know what they mean anyway
                    field = value
                }
                else -> {
                    field = clamp(value, LittleDebbie.shoulder.angles.min, LittleDebbie.shoulder.angles.max)
                }
            }
        }
    val shoulderAngle: Double
        get() {
            // we are using the encoder off a motor that is reversed, so negate the ticks
            val ticks = -shoulderEncoderMotor.currentPosition
            val ticksPerRevolution = 2468.5 // accounts for gear ratios etc. (empirically measured)
            val currentAngle = LittleDebbie.shoulder.angles.start + (ticks / ticksPerRevolution) * 360.0
            return currentAngle
        }
    var targetElbowAngle = LittleDebbie.elbow.angles.start
        private set(value) {
            if(!field.epsilonEquals(value))
                atTargetTimer.resetTimer()
            field = value
        }

    val elbowAngle: Double
        get() {
            // these were calculated empirically.
            val vlm = 1.30E+02
            val vlb = -1.30E+02
            val vrm = -1.31E+02
            val vrb = 2.96E+02
            val leftAngle =  leftElbowPosition.voltage * vlm + vlb + LittleDebbie.elbow.angleOffset
            val rightAngle = rightElbowPosition.voltage * vrm + vrb + LittleDebbie.elbow.angleOffset
            return (leftAngle + rightAngle) / 2.0
        }

    val isHome : Boolean
        get() = !magLimit.state
    val isAtTargetState : Boolean
        get() = abs(shoulderAngle - targetShoulderAngle) < LittleDebbie.shoulder.targetTolerance
                && abs(elbowAngle - targetElbowAngle) < LittleDebbie.elbow.targetTolerance
    var pidPower : Double = 0.0
        private set
    var elbowFeedforward : Double = 0.0
        private set
    var shoulderFeedforward : Double = 0.0
        private set
    val totalFeedforward : Double
        get() = elbowFeedforward + shoulderFeedforward
    val totalPower
        get() = clamp(pidPower + totalFeedforward, -LittleDebbie.shoulder.maxPower, LittleDebbie.shoulder.maxPower)
    private val servoAngleSlewRateFilter = SlewRateFilter(LittleDebbie.elbow.angleSlewLimit)
    private val atTargetTimer = Timer()
    val secondsAtTarget
        get() = atTargetTimer.elapsedTimeSeconds

    fun init() {
        // shoulder
        shoulderMotor = hardwareMap.get<DcMotorSimple>(DcMotorSimple::class.java, "shoulder")
        shoulderEncoderMotor = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, FollowerConstants.leftFrontMotorName)

        // ensure that the motor direction is set properly even when the follower is not running (e.g. a test opmode)
        shoulderEncoderMotor.direction = FollowerConstants.leftFrontMotorDirection
        shoulderMotor.direction = DcMotorSimple.Direction.REVERSE

        shoulderMotor.power = 0.0
        resetShoulderEncoder()
        shoulderPID.reset()

        // Elbow
        leftElbowServo = hardwareMap.get<ServoImplEx>(ServoImplEx::class.java, "leftElbow")
        rightElbowServo = hardwareMap.get<ServoImplEx>(ServoImplEx::class.java, "rightElbow")

        leftElbowPosition = hardwareMap.get<AnalogInput>(AnalogInput::class.java, "leftElbowPosition")
        rightElbowPosition = hardwareMap.get<AnalogInput>(AnalogInput::class.java, "rightElbowPosition")

        rightElbowServo.direction = Servo.Direction.REVERSE


        // mag limit
        magLimit = hardwareMap.get<DigitalChannel>(DigitalChannel::class.java, "maglimit")
    }
    private fun resetShoulderEncoder() {
        shoulderEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        shoulderEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }
    fun update() {
        updateTargetsForArmState(state)

        if(!isAtTargetState) atTargetTimer.resetTimer()

        // shoulder PID
        shoulderFeedforward = LittleDebbie.shoulder.Kf * cos(Math.toRadians(shoulderAngle))
        elbowFeedforward = LittleDebbie.elbow.Kf * cos(Math.toRadians(elbowAngle))

        val error = targetShoulderAngle - shoulderAngle
        shoulderPID.updateError(error)
        pidPower = shoulderPID.runPIDF()

        shoulderMotor.power = totalPower

        // elbow servos
        val filteredServoAngle = servoAngleSlewRateFilter.filter(targetElbowAngle)
        val position = servoAngleToPosition(filteredServoAngle)
        leftElbowServo.position = position
        rightElbowServo.position = position
    }
    fun beginHoming(){
        Log.d("ARM","beginHoming")
        state = ArmStates.HOMING
        targetShoulderAngle = shoulderAngle
    }
    fun moveUp() {
        Log.d("ARM","moveUp")
        Log.d("ARM","state = $state")
        Log.d("ARM","isHome = $isHome")
        Log.d("ARM","target pre = $targetShoulderAngle")
        Log.d("ARM","increment = ${LittleDebbie.shoulder.angles.homingAngleIncrement}")

        // if homing move up
        if(state == ArmStates.HOMING && !isHome)
            targetShoulderAngle += LittleDebbie.shoulder.angles.homingAngleIncrement
        Log.d("ARM","target post = $targetShoulderAngle")
    }
    fun moveDown() {
        Log.d("ARM","moveDown")
        if(state == ArmStates.HOMING && !isHome)
            targetShoulderAngle -= LittleDebbie.shoulder.angles.homingAngleIncrement
    }
    fun resetHome() {
        // NOTE: this resets home no matter whether you detected isHome or not
        if(state == ArmStates.HOMING) {
            resetShoulderEncoder()
            targetShoulderAngle = LittleDebbie.shoulder.angles.start
        }
    }
    private fun servoAngleToPosition(angle: Double) : Double {
        // derived empirically
        val m = -2.61E-03
        val b = 7.22E-01
        return m * (angle - LittleDebbie.elbow.angleOffset) + b
    }
    private fun updateTargetsForArmState(state: ArmStates) {
        when (state) {
            ArmStates.HOMING -> {
                targetElbowAngle = LittleDebbie.elbow.angles.start
            }
            ArmStates.START -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.start
                targetElbowAngle = LittleDebbie.elbow.angles.start
            }
            ArmStates.TRAVEL -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.travel
                targetElbowAngle = LittleDebbie.elbow.angles.travel
            }
            ArmStates.BELT_INSTALLATION -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.beltInstallation
                targetElbowAngle = LittleDebbie.elbow.angles.beltInstallation
            }
            ArmStates.SAMPLE_PREINTAKE -> {
                val tolerance = 5.0
                targetShoulderAngle = LittleDebbie.shoulder.angles.samplePreIntake
                if(abs(shoulderAngle - targetShoulderAngle) < tolerance)
                    targetElbowAngle = LittleDebbie.elbow.angles.samplePreIntake
            }
            ArmStates.SAMPLE_INTAKE -> {
                val tolerance = 5.0
                targetShoulderAngle = LittleDebbie.shoulder.angles.sampleIntake
                if(abs(shoulderAngle - targetShoulderAngle) < tolerance)
                    targetElbowAngle = LittleDebbie.elbow.angles.sampleIntake
            }
            ArmStates.SAMPLE_PREPOSITION_INTAKE -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.samplePrePositionIntake
                targetElbowAngle = LittleDebbie.elbow.angles.samplePrePositionIntake
            }
            ArmStates.SAMPLE_SCORE_BUCKET_HIGH -> {
                // move the elbow first then the shoulder
                targetElbowAngle = LittleDebbie.elbow.angles.sampleScoreHigh

                val tolerance = 5.0
                if(abs(elbowAngle - targetElbowAngle) < tolerance)
                    targetShoulderAngle = LittleDebbie.shoulder.angles.sampleScoreHigh
            }
            ArmStates.SAMPLE_SCORE_BUCKET_LOW -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.sampleScoreLow
                targetElbowAngle = LittleDebbie.elbow.angles.sampleScoreLow
            }
            ArmStates.SPECIMEN_INTAKE_WALL -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.specimenIntakeWall
                targetElbowAngle = LittleDebbie.elbow.angles.specimenIntakeWall
            }
            ArmStates.SPECIMEN_INTAKE_WALL_LIFTED -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.specimenIntakeWallLifted
                targetElbowAngle = LittleDebbie.elbow.angles.specimenIntakeWallLifted
            }
            ArmStates.SPECIMEN_ABOVE_HIGH_CHAMBER -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.specimenAboveHigh
                targetElbowAngle = LittleDebbie.elbow.angles.specimenAboveHigh
            }
            ArmStates.SPECIMEN_ABOVE_LOW_CHAMBER -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.specimenAboveLow
                targetElbowAngle = LittleDebbie.elbow.angles.specimenAboveLow
            }
            ArmStates.SPECIMEN_SCORE_HIGH_CHAMBER -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.specimenScoreHigh
                targetElbowAngle = LittleDebbie.elbow.angles.specimenScoreHigh
            }
            ArmStates.SPECIMEN_SCORE_LOW_CHAMBER -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.specimenScoreLow
                targetElbowAngle = LittleDebbie.elbow.angles.specimenScoreLow
            }
            ArmStates.PARK_OBSERVATION -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.parkObservation
                targetElbowAngle = LittleDebbie.elbow.angles.parkObservation
            }
            ArmStates.PARK_TOUCH_BAR -> {
                targetShoulderAngle = LittleDebbie.shoulder.angles.parkTouchBar
                targetElbowAngle = LittleDebbie.elbow.angles.parkTouchBar
            }
        }
    }

}