package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.teamcode.opmodes.config.LittleDebbie
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeStates
import kotlin.random.Random


@TeleOp(name = "Simple Intake Servo Tester", group = "Test")
class SimpleIntakeServoTest : OpMode() {
    private lateinit var intakeServo: ServoImplEx
    private lateinit var mt: MultipleTelemetry

    override fun init() {
        mt = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        intakeServo = hardwareMap.get<ServoImplEx>(ServoImplEx::class.java, "intake")
    }

    override fun loop() {

        // generate a random boolean

        if(gamepad1.a)
            intakeServo.position = LittleDebbie.intake.speeds.intake

        if(gamepad1.b)
            intakeServo.position = LittleDebbie.intake.speeds.outtake

        if(gamepad1.x)
            intakeServo.position = LittleDebbie.intake.speeds.stop

        if(gamepad1.y)
            intakeServo.position = LittleDebbie.intake.speeds.hold

        mt.addLine("> A to intake, B to outtake, X to stop, Y to Hold")
        mt.addData("Intake Position", intakeServo.position)
        mt.update()

    }

}