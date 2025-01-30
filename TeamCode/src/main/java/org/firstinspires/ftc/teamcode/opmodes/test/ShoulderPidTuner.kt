package org.firstinspires.ftc.teamcode.opmodes.test

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import com.qualcomm.robotcore.eventloop.opmode.TeleOp
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ArmStates
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer
import org.firstinspires.ftc.teamcode.util.ButtonReader

@TeleOp(name = "Shoulder PID Tuner", group = "Test")
class ShoulderPidTuner : OpMode() {
    private lateinit var arm : ArmSubsystem
    private lateinit var mt: MultipleTelemetry
    private lateinit var allHubs : List<LynxModule>
    private val loopTimer = Timer()
    private lateinit var crossButton : ButtonReader
    private lateinit var circleButton : ButtonReader
    private lateinit var squareButton : ButtonReader
    private lateinit var triangleButton : ButtonReader


    override fun init() {
        mt = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)

        allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        crossButton = ButtonReader({ gamepad1.cross })
        circleButton = ButtonReader({ gamepad1.circle })
        squareButton = ButtonReader({ gamepad1.square })
        triangleButton = ButtonReader({ gamepad1.triangle })

        arm = ArmSubsystem(hardwareMap)
        arm.init()
    }

    override fun loop() {
        clearBulkCache()

        arm.state = when {
            crossButton.wasJustPressed -> ArmStates.TRAVEL
            circleButton.wasJustReleased -> ArmStates.SAMPLE_PREINTAKE
            squareButton.wasJustPressed -> ArmStates.SPECIMEN_ABOVE_HIGH_CHAMBER
            triangleButton.wasJustPressed -> ArmStates.SAMPLE_SCORE_BUCKET_HIGH
            else -> arm.state
        }

        arm.update()
        updateTelemetry()
    }
    private fun clearBulkCache() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }
    }
    private fun updateTelemetry() {
        val loopTime = loopTimer.elapsedTime
        loopTimer.resetTimer()
        mt.addData("loop time (ms)", loopTime)
        mt.addData("0 State", arm.state)
        mt.addData("1 Shoulder Target", arm.targetShoulderAngle)
        mt.addData("2 Shoulder Angle", arm.shoulderAngle)
        mt.addData("3 Elbow Target", arm.targetElbowAngle)
        mt.addData("4 Elbow Angle", arm.elbowAngle)
        mt.addData("5 PID Power ", arm.pidPower)
        mt.addData("6 Elbow Feedforward ", arm.elbowFeedforward)
        mt.addData("7 Shoulder Feedforward ", arm.shoulderFeedforward)
        mt.addData("8 Total Feedforward ", arm.totalFeedforward)
        mt.addData("9 Total Power ", arm.totalPower)
        mt.update()
    }


}