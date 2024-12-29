package org.firstinspires.ftc.teamcode.opmodes.teleop

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.ArmPositions
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.ArmSubsystem
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.IntakeStates
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose

@TeleOp(name = "Field-Relative Teleop", group = "Teleop")
class FieldRelativeTeleop : OpMode() {
    private lateinit var follower : Follower
    private lateinit var arm : ArmSubsystem
    private val startPose = Pose(0.0, 0.0, 0.0)

    /** This method is call once when init is played, it initializes the follower and subsystems  */
    override fun init() {
        follower = Follower(hardwareMap)
        follower.setStartingPose(startPose)
        arm = ArmSubsystem(hardwareMap)
        arm.init()
    }

    /** This method is called continuously after Init while waiting to be started.  */
    override fun init_loop() {
    }

    /** This method is called once at the start of the OpMode.  */
    override fun start() {
        follower.startTeleopDrive()
    }

    /** This is the main loop of the opmode and runs continuously after play  */
    override fun loop() {
        /* Update Pedro to move the robot based on:
                - Forward/Backward Movement: -gamepad1.left_stick_y
                - Left/Right Movement: -gamepad1.left_stick_x
                - Turn Left/Right Movement: -gamepad1.right_stick_x
                - Robot-Centric Mode: false
                */

        follower.setTeleOpMovementVectors(
            -gamepad1.left_stick_y.toDouble(),
            -gamepad1.left_stick_x.toDouble(),
            -gamepad1.right_stick_x.toDouble(),
            false
        )
        follower.update()
        arm.update()

       // TODO: this is just a sample, make it whatever you want
        if (gamepad1.a) {
            arm.targetPosition = ArmPositions.SAMPLE_INTAKE
        }

        if (gamepad1.b) {
            arm.targetPosition = ArmPositions.SAMPLE_INTAKE
        }

        if (gamepad1.right_bumper) {
            when(arm.intakeState) {
                IntakeStates.INTAKING -> {
                    arm.intakeState = IntakeStates.STOPPED
                }
                IntakeStates.OUTTAKING -> {
                    arm.intakeState = IntakeStates.STOPPED
                }
                IntakeStates.STOPPED -> {
                    arm.intakeState = IntakeStates.INTAKING
                }
            }
        }

        /* This could be paired with a PIDF to set the target position of the lift in teleop.
         * For this, you would have to update the lift pid and make sure to initializes the lift subsystem.
         */

        /*
        if (gamepad1.left_trigger > 0.5) {
            lift.setTarget(lTarget-50);
        }

        if (gamepad1.right_trigger > 0.5) {
            lift.setTarget(lTarget+50);
        }
        */

        /* Telemetry Outputs of our Follower */
        telemetry.addData("X", follower.pose.x)
        telemetry.addData("Y", follower.pose.y)
        telemetry.addData("Heading in Degrees", Math.toDegrees(follower.pose.heading))

        /* Telemetry Outputs of our ArmSubsystem */
        telemetry.addData("Arm Target Position", arm.targetPosition.toString())
        telemetry.addData("Intake State", arm.intakeState.toString())
        telemetry.addData("Intake Element", arm.intakeElement.toString())

        /* Update Telemetry to the Driver Hub */
        telemetry.update()
    }

    /** We do not use this because everything automatically should disable  */
    override fun stop() {
    }
}