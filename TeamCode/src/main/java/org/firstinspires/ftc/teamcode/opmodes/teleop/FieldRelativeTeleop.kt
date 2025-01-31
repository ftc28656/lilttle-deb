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
import org.firstinspires.ftc.teamcode.util.ButtonReader
import org.firstinspires.ftc.teamcode.util.SlewRateFilter
import org.firstinspires.ftc.teamcode.util.TriggerReader

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

    private val xSlewRateFilter = SlewRateFilter(LittleDebbie.drive.translationSlew)
    private val ySlewRateFilter = SlewRateFilter(LittleDebbie.drive.translationSlew)
    private val turnSlewRateFilter = SlewRateFilter(LittleDebbie.drive.rotationalSlew)

    private lateinit var allHubs : List<LynxModule>
    private val loopTimer = Timer()

    private var scoringMode = ScoringMode.SAMPLES
    private var scoringPosition = ScoringPosition.HIGH
    private var alliance = Alliance.RED
    private var telopState = TeleopState.TRAVEL

    private lateinit var dpadUp : ButtonReader
    private lateinit var dpadDown : ButtonReader
    private lateinit var dpadLeft : ButtonReader
    private lateinit var dpadRight : ButtonReader

    private lateinit var crossButton : ButtonReader
    private lateinit var circleButton : ButtonReader
    private lateinit var squareButton : ButtonReader
    private lateinit var triangleButton : ButtonReader

    private lateinit var rightBumper : ButtonReader
    private lateinit var leftBumper : ButtonReader
    private lateinit var leftTrigger : TriggerReader
    private lateinit var rightTrigger1 : TriggerReader
    private lateinit var rightTrigger2 : TriggerReader

    private lateinit var optionsButton : ButtonReader

    private val outtakeTimer = Timer()

    /** This method is call once when init is played, it initializes the follower and subsystems  */
    override fun init() {
        mt = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        dpadUp = ButtonReader({ gamepad1.dpad_up })
        dpadDown = ButtonReader({ gamepad1.dpad_down })
        dpadLeft = ButtonReader({ gamepad1.dpad_left })
        dpadRight = ButtonReader({ gamepad1.dpad_right })

        crossButton = ButtonReader({ gamepad1.cross })
        circleButton = ButtonReader({ gamepad1.circle })
        squareButton = ButtonReader({ gamepad1.square })
        triangleButton = ButtonReader({ gamepad1.triangle })

        rightBumper = ButtonReader({ gamepad1.right_bumper })
        leftBumper = ButtonReader({ gamepad1.left_bumper })
        rightTrigger1 = TriggerReader({ gamepad1.right_trigger }, 0.1)
        rightTrigger2 = TriggerReader({ gamepad1.right_trigger }, 0.9)
        leftTrigger = TriggerReader({ gamepad1.left_trigger }, 0.5)

        optionsButton = ButtonReader({ gamepad1.options })

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
            rightTrigger1.wasJustPressed -> IntakeStates.INTAKING
            leftTrigger.wasJustPressed -> IntakeStates.OUTTAKING
            else -> intake.state
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
            triangleButton.wasJustPressed -> {
                val alignmentPose = Pose(follower.pose.x, follower.pose.y, Math.toRadians(LittleDebbie.drive.alignmentHeading))
                follower.pose = alignmentPose
            }
            optionsButton.wasJustPressed-> {
                arm.beginHoming()
                telopState = TeleopState.HOMING
            }

        }

        // Now update the scoring state based on input
        telopState =  when(telopState) {
            TeleopState.TRAVEL -> when {
                dpadUp.wasJustPressed -> {
                    scoringPosition = ScoringPosition.HIGH
                    telopState
                }
                dpadDown.wasJustPressed -> {
                    scoringPosition = ScoringPosition.LOW
                    telopState
                }
                dpadLeft.wasJustPressed -> {
                    scoringMode = ScoringMode.SAMPLES
                    telopState
                }
                dpadRight.wasJustPressed -> {
                    scoringMode = ScoringMode.SPECIMEN
                    telopState
                }
                leftTrigger.wasJustPressed -> when(scoringMode) {
                            ScoringMode.SAMPLES ->  when (scoringPosition) {
                                    ScoringPosition.HIGH -> TeleopState.SAMPLE_HIGH
                                    ScoringPosition.LOW -> TeleopState.SAMPLE_LOW
                                }
                            ScoringMode.SPECIMEN -> when(scoringPosition) {
                                   ScoringPosition.HIGH -> TeleopState.SPECIMEN_HIGH
                                   ScoringPosition.LOW -> TeleopState.SPECIMEN_LOW
                                }
                        }
                rightTrigger1.wasJustPressed -> when(scoringMode) {
                    ScoringMode.SAMPLES -> TeleopState.SAMPLE_HOVER
                    ScoringMode.SPECIMEN -> TeleopState.SAMPLE_HOVER
                    }
                else -> TeleopState.TRAVEL
            }
            TeleopState.SAMPLE_HOVER -> when {
                rightTrigger1.wasJustReleased -> TeleopState.TRAVEL
                rightTrigger2.wasJustPressed -> TeleopState.SAMPLE_INTAKING
                leftTrigger.wasJustPressed -> {
                    intake.state = IntakeStates.OUTTAKING
                    TeleopState.SAMPLE_HOVER
                }
                else -> TeleopState.SAMPLE_HOVER
            }
            TeleopState.SAMPLE_INTAKING -> when {
                dpadUp.wasJustPressed -> {
                    LittleDebbie.shoulder.angles.sampleIntake += 1.0
                    telopState
                }
                dpadDown.wasJustPressed -> {
                    LittleDebbie.shoulder.angles.sampleIntake -= 1.0
                    telopState
                }
                dpadLeft.wasJustPressed -> {
                    LittleDebbie.elbow.angles.sampleIntake -= 1.0
                    telopState
                }
                dpadRight.wasJustPressed -> {
                    LittleDebbie.elbow.angles.sampleIntake += 1.0
                    telopState
                }
                rightTrigger2.wasJustReleased -> TeleopState.SAMPLE_HOVER
                else -> TeleopState.SAMPLE_INTAKING
            }
            TeleopState.SAMPLE_HIGH -> when {
                leftTrigger.wasJustReleased -> TeleopState.OUTTAKING
                rightBumper.wasJustPressed -> TeleopState.TRAVEL
                leftBumper.wasJustPressed -> {
                    scoringPosition = ScoringPosition.LOW
                    TeleopState.SAMPLE_LOW
                }
                else -> TeleopState.SAMPLE_HIGH
            }
            TeleopState.SAMPLE_LOW -> when {
                leftTrigger.wasJustReleased -> TeleopState.OUTTAKING
                leftBumper.wasJustPressed -> {
                    scoringPosition = ScoringPosition.HIGH
                    TeleopState.SAMPLE_HIGH
                }
                else -> TeleopState.SAMPLE_LOW
            }
            TeleopState.SPECIMEN_HIGH -> when {
                leftTrigger.wasJustReleased -> TeleopState.OUTTAKING
                leftBumper.wasJustPressed -> {
                    scoringPosition = ScoringPosition.LOW
                    TeleopState.SPECIMEN_LOW
                }
                else -> TeleopState.SPECIMEN_HIGH
            }
            TeleopState.SPECIMEN_LOW -> when {
                leftTrigger.wasJustReleased -> TeleopState.OUTTAKING
                leftBumper.wasJustPressed -> {
                    scoringPosition = ScoringPosition.HIGH
                    TeleopState.SAMPLE_HIGH
                }
                else -> TeleopState.SPECIMEN_LOW
            }
            TeleopState.OUTTAKING -> when(scoringMode) {
                ScoringMode.SAMPLES -> when {
                    intake.element == IntakeElement.NONE -> TeleopState.TRAVEL
                    rightBumper.wasJustPressed -> TeleopState.TRAVEL
                    else -> TeleopState.OUTTAKING
                }
                ScoringMode.SPECIMEN -> when {
                    arm.isAtTargetState -> TeleopState.TRAVEL
                    rightTrigger1.wasJustPressed -> TeleopState.TRAVEL
                    leftTrigger.wasJustPressed -> when(scoringPosition) {
                        ScoringPosition.HIGH -> TeleopState.SPECIMEN_HIGH
                        ScoringPosition.LOW -> TeleopState.SPECIMEN_LOW
                    }
                    else -> TeleopState.OUTTAKING
                }
            }
            TeleopState.HOMING -> when {
                arm.isHome -> {
                    arm.resetHome()
                    TeleopState.TRAVEL
                }
                dpadUp.isPressed -> {
                    arm.moveUp()
                    TeleopState.HOMING
                }
                dpadDown.isPressed -> {
                    arm.moveDown()
                    TeleopState.HOMING
                }
                crossButton.wasJustPressed -> {
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
            else -> if(telopState == TeleopState.HOMING)
                        RearLightStates.INITIALIZING
                    else when(scoringMode) {
                        ScoringMode.SAMPLES -> RearLightStates.SAMPLE_MODE
                        ScoringMode.SPECIMEN -> RearLightStates.SPECIMEN_MODE
                        else -> RearLightStates.EMPTY_AND_OFF
                    }
        }
    }
    private fun updateTelemetry() {
        val loopTime = loopTimer.elapsedTime
        loopTimer.resetTimer()
        mt.addData("a loop time (ms)", loopTime)
        mt.addData("b X", follower.pose.x)
        mt.addData("c Y", follower.pose.y)
        mt.addData("d Heading (deg)", Math.toDegrees(follower.pose.heading))

        mt.addData("e Teleop State", telopState)
        mt.addData("f Scoring Mode", scoringMode)
        mt.addData("g Scoring Position", scoringPosition)

        mt.addData("h Arm State", arm.state)
        mt.addData("i Arm Shoulder Target Angle", arm.targetShoulderAngle)
        mt.addData("j Arm Shoulder Angle", arm.shoulderAngle)
        mt.addData("k Arm Elbow Target", arm.targetElbowAngle)
        mt.addData("l Arm Elbow Angle", arm.elbowAngle)
//        mt.addData("m Arm IsHome", arm.isHome)
//
//        mt.addData("n Intake State", intake.state)
//        mt.addData("o Intake Element", intake.element)
//        mt.addData("p Intake Distance", intake.distance)
//        mt.addData("q Intake HSV", intake.hsv.toString())
//
//        mt.addData("r Rear Light State", rearLight.state)
        mt.update()
    }
    /** We do not use this because everything automatically should disable  */
    override fun stop() {
    }
}