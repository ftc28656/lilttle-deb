package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.PwmControl
import com.qualcomm.robotcore.hardware.Servo
import com.qualcomm.robotcore.hardware.ServoController
import java.lang.Thread.sleep

@Disabled
@TeleOp(name = "Rear Light Tester", group = "Test")
class RearLightTester : OpMode() {
    lateinit var rearLight: Servo
    val range = PwmControl.PwmRange(500.0, 2500.0)
    var increment = 0.05
    var currentPos = 0.5

    override fun init() {
        rearLight = hardwareMap.get<Servo>(Servo::class.java, "rearLight")
        val control  =  (rearLight as PwmControl)

        // Set the PWM range
        telemetry.addData("lower (us)",control.pwmRange.usPulseLower )
        telemetry.addData("upper (us)",control.pwmRange.usPulseUpper )
        telemetry.addData(">", "Press Start.")
        telemetry.update()
    }

    override fun loop() {
        if (currentPos>1.0 || currentPos < 0.0)
            increment =- increment

        currentPos += increment
        rearLight.position = currentPos

        val pwmUs = range.usPulseLower + (range.usPulseUpper - range.usPulseLower) * rearLight.position
        telemetry.addData("Servo Pos", "%5.1f", rearLight.position)
        telemetry.addData("Servo Pulsewidth (us)", "%5.1f", pwmUs)
        telemetry.addData(">", "Press Stop to end test.")
        telemetry.update()
        telemetry.update()

        sleep(100)
    }
}