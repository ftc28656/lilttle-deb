package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.AnalogInput
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoImplEx
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptScanServo
import java.lang.Thread.sleep
import kotlin.math.max
import kotlin.math.min

@TeleOp(name = "Elbow Servo Tester", group = "Test")
class ElbowServoTester : OpMode() {
    var position = 0.5
    val increment = 0.002

    val homePosition = 0.40
    val scoreHighBucketPosition = 0.47
    val scoreMediumPosition = 0.32
    val farIntakePosition = 0.79

    lateinit var rightElbowServo: ServoImplEx
    lateinit var leftElbowServo: ServoImplEx
    lateinit var leftElbowPosition: AnalogInput
    lateinit var rightElbowPosition: AnalogInput
    private lateinit var mt: MultipleTelemetry


    override fun init() {
        mt = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        leftElbowServo = hardwareMap.get<ServoImplEx>(ServoImplEx::class.java, "leftElbow")
        rightElbowServo = hardwareMap.get<ServoImplEx>(ServoImplEx::class.java, "rightElbow")

        leftElbowPosition = hardwareMap.get<AnalogInput>(AnalogInput::class.java, "leftElbowPosition")
        rightElbowPosition = hardwareMap.get<AnalogInput>(AnalogInput::class.java, "rightElbowPosition")

        rightElbowServo.direction = Servo.Direction.REVERSE


        // Wait for the start button
        mt.addData("Left pwn lower (us)", leftElbowServo.pwmRange.usPulseLower)
        mt.addData("Left pwn upper (us)", leftElbowServo.pwmRange.usPulseUpper)
        mt.addData("right pwn lower (us)", rightElbowServo.pwmRange.usPulseLower)
        mt.addData("right pwn upper (us)", rightElbowServo.pwmRange.usPulseUpper)
        mt.addData(">", "Press Start.")
        mt.update()    }

    override fun loop() {

        if(gamepad1.left_bumper)
            position = min(1.0, position + increment)

        if(gamepad1.right_bumper)
            position = max(0.0, position - increment)

//        if (gamepad1.a)
//            position = homePosition
//        if (gamepad1.b)
//            position = scoreHighBucketPosition
//        if (gamepad1.x)
//            position = scoreMediumPosition
//        if (gamepad1.y)
//            position = farIntakePosition

        // Display the current value
        mt.addLine("> left bummper down")
        mt.addLine("> right bummper up")
        mt.addData("Servo Position", "%5.2f", position)
        mt.addData("left Analog", leftElbowPosition.voltage)
        mt.addData("right Analog", rightElbowPosition.voltage)
        mt.addData("Angle from Position (deg)", getCommandedAngle(position))
        mt.addData("Angle from 4th wire (deg)", getAngleFrom4thWireVoltage(leftElbowPosition.voltage, rightElbowPosition.voltage))
        mt.addData(">", "Press Stop to end test.")
        mt.update()

        // Set the servo to the new position and pause;
        leftElbowServo.position = position
        rightElbowServo.position = position

        sleep(50)
    }

    // returns degrees
    private fun getCommandedAngle(position: Double) : Double {
        val m = -3.91e2
        val b = 2.90e2
        return m * position + b
    }
    private fun getAngleFrom4thWireVoltage(leftVoltage : Double, rightVoltage : Double) : Double {
        val vlm = 1.32E+02
        val vlb = -1.22E+02
        val vrm = -1.32E+02
        val vrb = 3.10E+02
        val leftAngle =  leftVoltage * vlm + vlb
        val rightAngle = rightVoltage * vrm + vrb
        return (leftAngle + rightAngle) / 2.0
    }


}