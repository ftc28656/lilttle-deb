package org.firstinspires.ftc.teamcode.opmodes.test

import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DigitalChannel
import com.qualcomm.robotcore.hardware.ServoImplEx

@TeleOp(name = "Mag Limit Tester", group = "Test")
class MagLimitTester : OpMode() {
    lateinit var magLimit: DigitalChannel

    override fun init() {
        magLimit = hardwareMap.get<DigitalChannel>(DigitalChannel::class.java, "maglimit")
    }

    override fun loop() {
        // note active low (false is triggered)
        if(magLimit.state)
            telemetry.addData("magLimit", "not triggered")
        else
            telemetry.addData("magLimit", "triggered")

        telemetry.update()
    }
}