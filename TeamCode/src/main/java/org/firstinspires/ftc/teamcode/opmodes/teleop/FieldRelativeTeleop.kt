package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
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


@TeleOp(name = "Field-Relative Teleop", group = "Teleop")
class FieldRelativeTeleop : OpMode() {
    private lateinit var follower : Follower
    private val startPose = Pose(7.5, 112.5, Math.toRadians(270.0))

    private lateinit var arm : ArmSubsystem
    private lateinit var rearLight : RearLightSubsystem
    private lateinit var intake : IntakeSubsysten

    private lateinit var mt: MultipleTelemetry

    private val xSlewRateFilter = SlewRateFilter(0.5)
    private val ySlewRateFilter = SlewRateFilter(0.5)
    private val turnSlewRateFilter = SlewRateFilter(0.5)

    private lateinit var allHubs : List<LynxModule>
    private val loopTimer = Timer()

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

        if(gamepad1.right_bumper) {
            intake.state = when(intake.element) {
                IntakeElement.NONE -> IntakeStates.INTAKING
                else -> IntakeStates.OUTTAKING
            }
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
        val x = xSlewRateFilter.filter(-gamepad1.left_stick_x.toDouble())
        val y = ySlewRateFilter.filter(-gamepad1.left_stick_y.toDouble())
        val turn = turnSlewRateFilter.filter(-gamepad1.right_stick_x.toDouble())
        follower.setTeleOpMovementVectors(x,y,turn,false)
        follower.update()

        // udpdate the arm state based on the gamepad inputs\
        when {
            gamepad1.cross -> {
                arm.state = ArmStates.SAMPLE_PREINTAKE
                intake.state = IntakeStates.STOPPED
            }
            gamepad1.circle -> {
                arm.state = ArmStates.SAMPLE_INTAKE
                intake.state = IntakeStates.INTAKING
            }
            gamepad1.square -> arm.state = ArmStates.SAMPLE_SCORE_BUCKET_HIGH
            gamepad1.triangle -> arm.state = ArmStates.SAMPLE_SCORE_BUCKET_LOW
            gamepad1.right_bumper -> intake.state = IntakeStates.INTAKING
            gamepad1.left_bumper -> intake.state = IntakeStates.OUTTAKING
            gamepad1.dpad_down -> arm.state = ArmStates.SPECIMEN_INTAKE_WALL
            gamepad1.dpad_up ->  arm.state = ArmStates.SPECIMEN_INTAKE_WALL_LIFTED
            gamepad1.dpad_left -> arm.state = ArmStates.SPECIMEN_ABOVE_HIGH_CHAMBER
            gamepad1.dpad_right -> arm.state = ArmStates.SPECIMEN_SCORE_HIGH_CHAMBER

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