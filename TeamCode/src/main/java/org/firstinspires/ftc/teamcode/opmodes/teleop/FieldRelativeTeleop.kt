package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.config.LittleDebbie
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ArmStates
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeElement
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeStates
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeSubsysten
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.rearlight.RearLightStates
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.rearlight.RearLightSubsystem
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer
import org.firstinspires.ftc.teamcode.util.SlewRateFilter

enum class ScoringMode {
    SAMPLES,
    SPECIMEN
}
enum class ScoringPosition {
    HIGH,
    LOW
}
enum class Alliance {
    RED,
    BLUE
}
enum class TeleopState {
    TRAVEL,
    HOMING,
    SAMPLE_HOVER,
    SAMPLE_INTAKING,
    SAMPLE_HIGH,
    SAMPLE_LOW,
    SPECIMEN_HIGH,
    SPECIMEN_LOW,
    OUTTAKING,
}

@TeleOp(name = "Field-Relative Teleop", group = "Teleop")
class FieldRelativeTeleop : OpMode() {
    private lateinit var follower : Follower
    private val startPose = Pose(7.5, 112.5, Math.toRadians(270.0))

    private lateinit var arm : ArmSubsystem
    private lateinit var rearLight : RearLightSubsystem
    private lateinit var intake : IntakeSubsysten

    private lateinit var mt: MultipleTelemetry

    private val xSlewRateFilter = SlewRateFilter({ LittleDebbie.drive.xMaxSlewRate })
    private val ySlewRateFilter = SlewRateFilter({ LittleDebbie.drive.yMaxSlewRate })
    private val turnSlewRateFilter = SlewRateFilter({ LittleDebbie.drive.turnMaxSlewRate })

    private lateinit var allHubs : List<LynxModule>
    private val loopTimer = Timer()

    private var scoringMode = ScoringMode.SAMPLES
    private var scoringPosition = ScoringPosition.HIGH
    private var alliance = Alliance.RED
    private var telopState = TeleopState.TRAVEL
    private val triggerThreshold = 0.5

    /** This method is call once when init is played, it initializes the follower and subsystems  */
    override fun init() {
        mt = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        follower = Follower(hardwareMap)
        follower.setStartingPose(startPose)

        arm = ArmSubsystem(hardwareMap)
        arm.init()

        rearLight = RearLightSubsystem(hardwareMap)
        rearLight.init()

        intake = IntakeSubsysten(hardwareMap)
        intake.init()
    }

    /** This method is called continuously after Init while waiting to be started.  */
    override fun init_loop() {
        clearBulkCache()

        intake.state = when {
            gamepad1.right_trigger > triggerThreshold -> IntakeStates.INTAKING
            gamepad1.left_trigger > triggerThreshold -> IntakeStates.OUTTAKING
            else -> IntakeStates.STOPPED
        }

        updateRearLight()
        arm.update()
        rearLight.update()
        intake.update()
        updateTelemetry()
    }

    /** This method is called once at the start of the OpMode.  */
    override fun start() {
        follower.startTeleopDrive()
    }

    /** This is the main loop of the opmode and runs continuously after play  */
    override fun loop() {
        clearBulkCache()

        // update the follower with the gamepad inputs with slew rate limiting
        // (https://docs.wpilib.org/en/stable/docs/software/advanced-controls/filters/slew-rate-limiter.html)
        val x = xSlewRateFilter.filter(-gamepad1.left_stick_y.toDouble())
        val y = ySlewRateFilter.filter(-gamepad1.left_stick_x.toDouble())
        val turn = turnSlewRateFilter.filter(-gamepad1.right_stick_x.toDouble())
        follower.setTeleOpMovementVectors(x,y,turn,false)
        follower.update()

        // update scoring mode and position based on the dpad and allow non-state-dependent things
        when {
            gamepad1.dpad_up -> scoringPosition = ScoringPosition.HIGH
            gamepad1.dpad_down -> scoringPosition = ScoringPosition.LOW
            gamepad1.dpad_left -> scoringMode = ScoringMode.SAMPLES
            gamepad1.dpad_right -> scoringMode = ScoringMode.SPECIMEN
            gamepad1.triangle -> {
                val alignmentPose = Pose(follower.pose.x, follower.pose.y, Math.toRadians(LittleDebbie.drive.alignmentHeading))
                follower.setStartingPose(alignmentPose)
            }
        }

        // Now update the scoring state based on input
        telopState =  when(telopState) {
            TeleopState.TRAVEL -> when {
                    gamepad1.right_bumper -> when(scoringMode) {
                            ScoringMode.SAMPLES -> when (intake.element) {
                                    IntakeElement.NONE -> TeleopState.SAMPLE_HOVER
                                    else -> when (scoringPosition) {
                                            ScoringPosition.HIGH -> TeleopState.SAMPLE_HIGH
                                            ScoringPosition.LOW -> TeleopState.SAMPLE_LOW
                                        }
                                }
                            ScoringMode.SPECIMEN -> when(scoringPosition) {
                                   ScoringPosition.HIGH -> TeleopState.SPECIMEN_HIGH
                                   ScoringPosition.LOW -> TeleopState.SPECIMEN_LOW
                                }
                        }
                    gamepad1.options -> TeleopState.HOMING
                    else -> TeleopState.TRAVEL
                }
            TeleopState.SAMPLE_HOVER -> when {
                gamepad1.right_bumper -> TeleopState.TRAVEL
                gamepad1.right_trigger > triggerThreshold -> TeleopState.SAMPLE_INTAKING
                else -> TeleopState.SAMPLE_HOVER
            }
            TeleopState.SAMPLE_INTAKING -> when {
                // TODO: add the ability to trim this somehow
                gamepad1.right_trigger < triggerThreshold -> TeleopState.SAMPLE_HOVER
                else -> TeleopState.SAMPLE_INTAKING
            }
            TeleopState.SAMPLE_HIGH -> when {
                gamepad1.right_trigger > triggerThreshold -> TeleopState.OUTTAKING
                gamepad1.right_bumper -> TeleopState.TRAVEL
                else -> TeleopState.SAMPLE_HIGH
            }
            TeleopState.SAMPLE_LOW -> when {
                gamepad1.right_trigger > triggerThreshold -> TeleopState.OUTTAKING
                gamepad1.right_bumper -> TeleopState.TRAVEL
                else -> TeleopState.SAMPLE_LOW
            }
            TeleopState.SPECIMEN_HIGH -> when {
                gamepad1.right_trigger > triggerThreshold -> TeleopState.OUTTAKING
                gamepad1.right_bumper -> TeleopState.TRAVEL
                else -> TeleopState.SPECIMEN_HIGH
            }
            TeleopState.SPECIMEN_LOW -> when {
                gamepad1.right_trigger > triggerThreshold -> TeleopState.OUTTAKING
                gamepad1.right_bumper -> TeleopState.TRAVEL
                else -> TeleopState.SPECIMEN_LOW
            }
            TeleopState.OUTTAKING -> when(scoringMode) {
                ScoringMode.SAMPLES -> when {
                    gamepad1.right_trigger < triggerThreshold -> TeleopState.TRAVEL
                    gamepad1.right_bumper -> TeleopState.TRAVEL
                    else -> TeleopState.OUTTAKING
                }
                ScoringMode.SPECIMEN -> when {
                    gamepad1.right_trigger < triggerThreshold -> TeleopState.TRAVEL
                    gamepad1.right_bumper -> TeleopState.TRAVEL
                    else -> TeleopState.OUTTAKING
                }
            }
            TeleopState.HOMING -> when {
                arm.isHome -> {
                    arm.resetHome()
                    TeleopState.TRAVEL
                }
                gamepad1.dpad_up -> {
                    arm.moveUp()
                    TeleopState.HOMING
                }
                gamepad1.dpad_down -> {
                    arm.moveDown()
                    TeleopState.HOMING
                }
                gamepad1.options -> {
                    arm.resetHome()
                    TeleopState.TRAVEL
                }
                else -> TeleopState.HOMING
            }
        }

        // update the arm and intake states based on the teleop state
        when(telopState) {
            TeleopState.TRAVEL -> {
                arm.state = ArmStates.TRAVEL
                intake.state = IntakeStates.STOPPED
            }
            TeleopState.HOMING -> {
                arm.state = ArmStates.HOMING
                intake.state = IntakeStates.STOPPED
            }
            TeleopState.SAMPLE_LOW -> {
                arm.state = ArmStates.SAMPLE_SCORE_BUCKET_LOW
            }
            TeleopState.SAMPLE_HIGH -> {
                arm.state = ArmStates.SAMPLE_SCORE_BUCKET_HIGH
            }
            TeleopState.OUTTAKING -> when(scoringMode) {
                ScoringMode.SAMPLES -> intake.state = IntakeStates.OUTTAKING
                ScoringMode.SPECIMEN -> when(scoringPosition) {
                    ScoringPosition.HIGH -> arm.state = ArmStates.SPECIMEN_SCORE_HIGH_CHAMBER
                    ScoringPosition.LOW -> arm.state = ArmStates.SPECIMEN_SCORE_LOW_CHAMBER
                }
            }
            TeleopState.SPECIMEN_HIGH -> arm.state = ArmStates.SPECIMEN_ABOVE_HIGH_CHAMBER
            TeleopState.SPECIMEN_LOW -> arm.state = ArmStates.SPECIMEN_ABOVE_HIGH_CHAMBER
            TeleopState.SAMPLE_HOVER -> {
                arm.state = ArmStates.SAMPLE_PREINTAKE
                intake.state = IntakeStates.STOPPED
            }
            TeleopState.SAMPLE_INTAKING -> {
                arm.state = ArmStates.SAMPLE_INTAKE
                intake.state = IntakeStates.INTAKING
            }
        }


        updateRearLight()
        arm.update()
        rearLight.update()
        intake.update()
        updateTelemetry()
    }
    private fun clearBulkCache() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }
    }
    private fun updateRearLight() {
        // TODO: update this based on teleop state, scoring mode, position
        rearLight.state = when(intake.element) {
            IntakeElement.RED -> RearLightStates.HOLDING_RED
            IntakeElement.YELLOW -> RearLightStates.HOLDING_YELLOW
            IntakeElement.BLUE -> RearLightStates.HOLDING_BLUE
            else -> when(intake.state) {
                IntakeStates.INTAKING -> RearLightStates.INTAKING
                IntakeStates.OUTTAKING -> RearLightStates.OUTTAKING
                else -> RearLightStates.EMPTY_AND_OFF
            }
        }
    }
    private fun updateTelemetry() {
        val loopTime = loopTimer.elapsedTime
        loopTimer.resetTimer()
        mt.addData("loop time (ms)", loopTime)
        mt.addData("X", follower.pose.x)
        mt.addData("Y", follower.pose.y)
        mt.addData("Heading (deg)", Math.toDegrees(follower.pose.heading))

        mt.addData("Teleop State", telopState)
        mt.addData("Scoring Mode", scoringMode)
        mt.addData("Teleop State", telopState)
        mt.addData("Alliance", alliance)

        mt.addData("Arm State", arm.state)
        mt.addData("Arm Shoulder Target Angle", arm.targetShoulderAngle)
        mt.addData("Arm Shoulder Angle", arm.shoulderAngle)
        mt.addData("Arm Elbow Target", arm.targetElbowAngle)
        mt.addData("Arm Elbow Angle", arm.elbowAngle)

        mt.addData("Intake State", intake.state)
        mt.addData("Intake Element", intake.element)
        mt.addData("Intake Distance", intake.distance)
        mt.addData("Intake HSV", intake.hsv.toString())

        mt.addData("Rear Light State", rearLight.state)
        mt.update()
    }
    /** We do not use this because everything automatically should disable  */
    override fun stop() {
    }
}