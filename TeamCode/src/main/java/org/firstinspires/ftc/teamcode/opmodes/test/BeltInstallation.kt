package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ArmStates
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ArmSubsystem


@TeleOp(name = "Belt Installation", group = "Test")
class BeltInstallation : OpMode() {
    lateinit var arm : ArmSubsystem
    private lateinit var mt: MultipleTelemetry

    override fun init() {
        mt = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        arm = ArmSubsystem(hardwareMap)
        arm.init()

    }

    override fun loop() {
        arm.state = ArmStates.BELT_INSTALLATION
        arm.update()

        mt.addData("Arm State", arm.state)
        mt.addData("Arm Shoulder Target Angle", arm.targetShoulderAngle)
        mt.addData("Arm Shoulder Angle", arm.shoulderAngle)
        mt.addData("Arm Elbow Target", arm.targetElbowAngle)
        mt.addData("Arm Elbow Angle", arm.elbowAngle)
        mt.update()
    }
}