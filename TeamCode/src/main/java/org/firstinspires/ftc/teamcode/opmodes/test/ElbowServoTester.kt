package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.Servo
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptScanServo
import java.lang.Thread.sleep
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "Elbow Servo Tester", group = "Test")
class ElbowServoTester : OpMode() {
    var position = 0.5
    val increment = 0.05
    lateinit var rightElbowServo: Servo
    lateinit var leftElbowServo: Servo
    lateinit var leftElbowPosition: AnalogInput
    lateinit var rightElbowPosition: AnalogInput

    override fun init() {
        leftElbowServo = hardwareMap.get<Servo>(Servo::class.java, "leftElbow")
        rightElbowServo = hardwareMap.get<Servo>(Servo::class.java, "rightElbow")

        leftElbowPosition = hardwareMap.get<AnalogInput>(AnalogInput::class.java, "leftElbowPosition")
        rightElbowPosition = hardwareMap.get<AnalogInput>(AnalogInput::class.java, "rightElbowPosition")

        rightElbowServo.direction = Servo.Direction.REVERSE

        // Wait for the start button
        telemetry.addData(">", "Press Start.")
        telemetry.update()    }

    override fun loop() {
        if(gamepad1.a)
            position = min(1.0, position+increment)

        if(gamepad1.b)
            position = max(0.0,position-increment)


        // Display the current value
        telemetry.addData("Servo Position", "%5.2f", position)
        telemetry.addData("left Analog", leftElbowPosition.voltage)
        telemetry.addData("right Analog", rightElbowPosition.voltage)
        telemetry.addData(">", "Press Stop to end test.")
        telemetry.update()

        // Set the servo to the new position and pause;
        leftElbowServo.position = position
        rightElbowServo.position = position

        sleep(50)
    }
}