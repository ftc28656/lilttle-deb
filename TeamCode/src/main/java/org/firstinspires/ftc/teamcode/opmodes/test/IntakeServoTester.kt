package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeStates
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeSubsysten
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer


@Disabled
@TeleOp(name = "Intake Servo Tester", group = "Test")
class IntakeServoTester : OpMode() {
    lateinit var intake: IntakeSubsysten
    private lateinit var mt: MultipleTelemetry

    override fun init() {
        mt = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        intake = IntakeSubsysten(hardwareMap)
        intake.init()
    }

    override fun loop() {

        if(gamepad1.a)
            intake.state = IntakeStates.INTAKING

        if(gamepad1.b)
            intake.state = IntakeStates.OUTTAKING

        if(gamepad1.x)
            intake.state = IntakeStates.STOPPED

        if(gamepad1.y)
            intake.state = IntakeStates.HOLDING_ELEMENT

        intake.update()

        mt.addLine("> A to intake, B to outtake, X to stop, Y to Hold")
        mt.addData("Intake State", intake.state.toString())
        mt.addData("Intake Element", intake.element.toString())
        mt.addData("Intake Servo Pos", intake.servoPosition)
        mt.addData("Intake color", intake.hsv)
        mt.addData("Intake distance", intake.distance)


        mt.update()


    }

}
