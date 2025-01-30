package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.Disabled
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import com.qualcomm.robotcore.hardware.DcMotor
import com.qualcomm.robotcore.hardware.DcMotorEx
import com.qualcomm.robotcore.hardware.DcMotorSimple
import org.firstinspires.ftc.teamcode.opmodes.config.LittleDebbie
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions.clamp
import org.firstinspires.ftc.teamcode.pedroPathing.tuning.FollowerConstants
import org.firstinspires.ftc.teamcode.pedroPathing.util.PIDFController

@Disabled
@TeleOp(name = "Shoulder Tester", group = "Test")
class ShoulderTester : OpMode() {

    lateinit var shoulderMotor : DcMotorSimple
    lateinit var shoulderEncoderMotor : DcMotorEx
    val mt = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

    var targetAngle = LittleDebbie.shoulder.angles.start
        set(value) {
            field = clamp(value, LittleDebbie.shoulder.angles.min, LittleDebbie.shoulder.angles.max)
        }
    val pid = PIDFController(LittleDebbie.shoulder.pid)

    override fun init() {
        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during robot configuration.
        shoulderMotor = hardwareMap.get<DcMotorSimple>(DcMotorSimple::class.java, "shoulder")
        shoulderEncoderMotor = hardwareMap.get<DcMotorEx>(DcMotorEx::class.java, FollowerConstants.leftFrontMotorName)

        shoulderMotor.direction = DcMotorSimple.Direction.REVERSE
        shoulderMotor.power = 0.0
        reset()
        pid.reset()
    }

    override fun loop() {
        targetAngle = TestConstants.testTargetShoulderAngle

        val ticks = shoulderEncoderMotor.currentPosition
        val ticksPerRevolution = 2468.5 // accounts for gear ratios etc. (empirically measured)

        var currentAngle = LittleDebbie.shoulder.angles.start + (ticks / ticksPerRevolution) * 360.0
        val error = targetAngle - currentAngle
        pid.updateError(error)
        val power = clamp(pid.runPIDF(), -1.0, 1.0)
        shoulderMotor.power = power

        mt.addData("target angle (deg)", targetAngle)
        mt.addData("angle (deg)", currentAngle)
        mt.addData("ticks", ticks)
        mt.addData("power", power)
        mt.update()
    }

    fun reset() {
        shoulderEncoderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER)
        shoulderEncoderMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER)
    }

}