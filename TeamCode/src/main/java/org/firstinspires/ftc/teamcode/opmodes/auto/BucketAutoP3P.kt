package org.firstinspires.ftc.teamcode.opmodes.auto

import com.acmerobotics.dashboard.FtcDashboard
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry
import com.qualcomm.hardware.lynx.LynxModule
import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.opmodes.config.LittleDebbie
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ArmStates
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ArmSubsystem
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeElement
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeStates
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeSubsysten
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.rearlight.RearLightStates
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.rearlight.RearLightSubsystem
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.MathFunctions.clamp
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer

@Autonomous(name = "Bucket Auto Preload+3+Park", group = "Auto")
class BucketAutoP3P : OpMode() {
    private lateinit var mt: MultipleTelemetry

    private lateinit var follower: Follower

    private var pathSegmentTimer = Timer()
    private var opmodeTimer = Timer()
    private var pathState = BucketPathStates.START
    private val loopTimer = Timer()

    private lateinit var allHubs : List<LynxModule>
    lateinit var arm: ArmSubsystem
    private lateinit var rearLight : RearLightSubsystem
    private lateinit var intake : IntakeSubsysten
    private val nominalVoltage = 12.0
    private var batteryVoltage : Double = nominalVoltage

    private val startPose = LittleDebbie.auto.bucket.startPose.toPose()
    private val scorePose = LittleDebbie.auto.bucket.scoringPose.toPose()

    private val lineUpSample1Pose = LittleDebbie.auto.bucket.lineUpSample1Pose.toPose()
    private val pickupSample1Pose = LittleDebbie.auto.bucket.pickupSample1Pose.toPose()

    private val lineupSample2Pose = LittleDebbie.auto.bucket.lineupSample2Pose.toPose()
    private val pickupSample2Pose = LittleDebbie.auto.bucket.pickupSample2Pose.toPose()

    private val lineupSample3Pose = LittleDebbie.auto.bucket.lineupSample3Pose.toPose()
    private val pickupSample3Pose = LittleDebbie.auto.bucket.pickupSample3Pose.toPose()

    private val parkPose = LittleDebbie.auto.bucket.parkPose.toPose()
    private val parkControl1 = LittleDebbie.auto.bucket.parkControl1.toPoint()
    private val parkControl2 = LittleDebbie.auto.bucket.parkControl2.toPoint()

    private lateinit var scorePreload: PathChain
    private lateinit var park: PathChain
    private lateinit var lineupSample1: PathChain
    private lateinit var pickupSample1: PathChain
    private lateinit var lineupSample2: PathChain
    private lateinit var pickupSample2: PathChain
    private lateinit var lineupSample3: PathChain
    private lateinit var pickupSample3: PathChain
    private lateinit var scoreSample1: PathChain
    private lateinit var scoreSample2: PathChain
    private lateinit var scoreSample3: PathChain

    private val pathSegmentElapsedTime
        get() = pathSegmentTimer.elapsedTimeSeconds

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts.  */
    private fun buildPaths() {
        /* There are two major types of paths components: BezierCurves and BezierLines.
                 *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
                 *    - Control points manipulate the curve between the start and end points.
                 *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
                 *    * BezierLines are straight, and require 2 points. There are the start and end points.
                 * Paths have can have heading interpolation: Constant, Linear, or Tangential
                 *    * Linear heading interpolation:
                 *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
                 *    * Constant Heading Interpolation:
                 *    - Pedro will maintain one heading throughout the entire path.
                 *    * Tangential Heading Interpolation:
                 *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
                 * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
                 * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        /* This is our scorePreload pathChain. We are using a BezierLine, which is a straight line. */
        scorePreload = follower.pathBuilder()
            .addPath(BezierLine(Point(startPose), Point(scorePose)))
            .setLinearHeadingInterpolation(startPose.heading, scorePose.heading)
            .build()

        lineupSample1 = follower.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(lineUpSample1Pose)))
            .setLinearHeadingInterpolation(scorePose.heading, lineUpSample1Pose.heading)
            .build()

        pickupSample1 = follower.pathBuilder()
            .addPath(BezierLine(Point(lineUpSample1Pose), Point(pickupSample1Pose)))
            .setLinearHeadingInterpolation(lineUpSample1Pose.heading, pickupSample1Pose.heading)
            .build()

        scoreSample1 = follower.pathBuilder()
            .addPath(BezierLine(Point(pickupSample1Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(pickupSample1Pose.heading, scorePose.heading)
            .build()

        lineupSample2 = follower.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(lineupSample2Pose)))
            .setLinearHeadingInterpolation(scorePose.heading, lineupSample2Pose.heading)
            .build()

        pickupSample2 = follower.pathBuilder()
            .addPath(BezierLine(Point(lineupSample2Pose), Point(pickupSample2Pose)))
            .setLinearHeadingInterpolation(lineupSample2Pose.heading, pickupSample2Pose.heading)
            .build()

        /* This is our scoreSample2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSample2 = follower.pathBuilder()
            .addPath(BezierLine(Point(pickupSample2Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(pickupSample2Pose.heading, scorePose.heading)
            .build()

        lineupSample3 = follower.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(lineupSample3Pose)))
            .setLinearHeadingInterpolation(scorePose.heading, lineupSample3Pose.heading)
            .build()

        pickupSample3 = follower.pathBuilder()
            .addPath(BezierLine(Point(lineupSample3Pose), Point(pickupSample3Pose)))
            .setLinearHeadingInterpolation(lineupSample3Pose.heading, pickupSample3Pose.heading)
            .build()

        scoreSample3 = follower.pathBuilder()
            .addPath(BezierLine(Point(pickupSample3Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(pickupSample3Pose.heading, scorePose.heading)
            .build()

        /* This is our park pathChain. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = follower.pathBuilder()
            .addPath(BezierCurve(Point(scorePose), parkControl1, parkControl2, Point(parkPose)))
            .setTangentHeadingInterpolation()
            .build()
    }

    /** This function is used to update the state machine for the autonomous.  To extend it, just add more states and transitions.
     * */
    private fun autonomousPathUpdate() {
        when (pathState) {
            BucketPathStates.START -> {
            stateTransition(BucketPathStates.SCORING_PRELOAD) {
                arm.state = ArmStates.SAMPLE_SCORE_BUCKET_HIGH
                follower.followPath(scorePreload, true)
                }
           }
            BucketPathStates.SCORING_PRELOAD -> {
                if (!follower.isBusy && armOk()) {
                    intake.state = IntakeStates.OUTTAKING
                    if(intake.element == IntakeElement.NONE)
                        stateTransition(BucketPathStates.LINEUP_SAMPLE_1) {
                            arm.state = ArmStates.SAMPLE_PREINTAKE
                            follower.followPath(lineupSample1, true)
                        }
                }
            }
            BucketPathStates.LINEUP_SAMPLE_1 -> {
                if (!follower.isBusy && armOk())
                    intake.state = IntakeStates.INTAKING
                arm.state = ArmStates.SAMPLE_INTAKE
                if(armOk())
                    stateTransition(BucketPathStates.PICKUP_SAMPLE_1) {
                        follower.followPath(pickupSample1, true)
                    }
            }
            BucketPathStates.PICKUP_SAMPLE_1 ->
                if (!follower.isBusy || intake.element != IntakeElement.NONE)
                    stateTransition(BucketPathStates.SCORING_SAMPLE_1) {
                        arm.state = ArmStates.SAMPLE_SCORE_BUCKET_HIGH
                        follower.followPath(scoreSample1, true)
                    }
            BucketPathStates.SCORING_SAMPLE_1 ->
                if(!follower.isBusy && armOk()) {
                    intake.state = IntakeStates.OUTTAKING
                    if(intake.element == IntakeElement.NONE)
                        stateTransition(BucketPathStates.LINEUP_SAMPLE_2) {
                            arm.state = ArmStates.SAMPLE_PREINTAKE
                            follower.followPath(lineupSample2, true)
                        }
                }
            BucketPathStates.LINEUP_SAMPLE_2 -> {
                if (!follower.isBusy && armOk())
                    intake.state = IntakeStates.INTAKING
                    arm.state = ArmStates.SAMPLE_INTAKE
                    if(armOk())
                        stateTransition(BucketPathStates.PICKUP_SAMPLE_2) {
                            follower.followPath(pickupSample2, true)
                        }
            }
            BucketPathStates.PICKUP_SAMPLE_2 ->
                if(!follower.isBusy || intake.element != IntakeElement.NONE)
                    stateTransition(BucketPathStates.SCORING_SAMPLE_2) {
                        arm.state = ArmStates.SAMPLE_SCORE_BUCKET_HIGH
                        follower.followPath(scoreSample2, true)
                    }
            BucketPathStates.SCORING_SAMPLE_2 ->
                if(!follower.isBusy && armOk()) {
                    intake.state = IntakeStates.OUTTAKING
                    if(intake.element == IntakeElement.NONE)
                        stateTransition(BucketPathStates.LINEUP_SAMPLE_3) {
                            arm.state = ArmStates.SAMPLE_PREINTAKE
                            follower.followPath(lineupSample3, true)
                        }
                }
            BucketPathStates.LINEUP_SAMPLE_3 -> {
                if (!follower.isBusy && armOk())
                    intake.state = IntakeStates.INTAKING
                    arm.state = ArmStates.SAMPLE_INTAKE
                    if(armOk())
                        stateTransition(BucketPathStates.PICKUP_SAMPLE_3) {
                            follower.followPath(pickupSample3, true)
                        }
            }
            BucketPathStates.PICKUP_SAMPLE_3 ->
                if(!follower.isBusy || intake.element != IntakeElement.NONE)
                    stateTransition(BucketPathStates.SCORING_SAMPLE_3) {
                        arm.state = ArmStates.SAMPLE_SCORE_BUCKET_HIGH
                        follower.followPath(scoreSample3, true)
                    }
            BucketPathStates.SCORING_SAMPLE_3 ->
                if(!follower.isBusy && armOk()) {
                    intake.state = IntakeStates.OUTTAKING
                    if(intake.element == IntakeElement.NONE)
                        stateTransition(BucketPathStates.PARKING) {
                            arm.state = ArmStates.TRAVEL
                            follower.followPath(park, true)
                        }
                }
            BucketPathStates.PARKING -> {
                if(!follower.isBusy)
                    arm.state = ArmStates.PARK_TOUCH_BAR
            }
        }
    }

    private fun armOk() : Boolean {
        return arm.secondsAtTarget > LittleDebbie.auto.armSettleTime ||
                pathSegmentElapsedTime > LittleDebbie.auto.armTargetTimeOut
    }

    /** This function is used to transition between states in the state machine
     * It will also reset the pathSegmentTimer for each state   */
    private fun stateTransition(
        nextState: BucketPathStates,
        action: () -> Unit
    ) {
        action()
        pathState = nextState
        pathSegmentTimer.resetTimer()
    }


    override fun init() {
        mt = MultipleTelemetry(telemetry, FtcDashboard.getInstance().telemetry)
        opmodeTimer.resetTimer()

        allHubs = hardwareMap.getAll(LynxModule::class.java)
        for (hub in allHubs) {
            hub.bulkCachingMode = LynxModule.BulkCachingMode.MANUAL
        }

        follower = Follower(hardwareMap)
        follower.setStartingPose(startPose)

        buildPaths()

        arm = ArmSubsystem(hardwareMap)
        arm.init()

        rearLight = RearLightSubsystem(hardwareMap)
        rearLight.init()

        intake = IntakeSubsysten(hardwareMap)
        intake.init()
    }

    override fun init_loop() {
        clearBulkCache()
        updateRearLight()

        // scale the follower maxPower based on the batteryvoltage
        val alpha = 0.8  // for a low pass filter on battvoltage
        val newBatteryVoltage = hardwareMap.voltageSensor.first().voltage
        batteryVoltage = alpha * newBatteryVoltage + (1-alpha)* batteryVoltage
        val maxPowerAdjusted =  clamp(nominalVoltage / batteryVoltage,0.0, 1.0)
        follower.setMaxPower(maxPowerAdjusted * LittleDebbie.auto.maxPowerFraction)

        follower.update()
        arm.update()
        rearLight.update()
        intake.update()

        updateTelemetry()
    }

    override fun loop() {
        clearBulkCache()
        updateRearLight()

        // These loop the movements of the robot
        follower.update()
        arm.update()
        intake.update()
        rearLight.update()
        autonomousPathUpdate()

        updateTelemetry()
    }

    private fun clearBulkCache() {
        for (hub in allHubs) {
            hub.clearBulkCache()
        }
    }
    private fun updateRearLight() {
        rearLight.state = when(intake.element) {
            IntakeElement.RED -> RearLightStates.HOLDING_RED
            IntakeElement.YELLOW -> RearLightStates.HOLDING_YELLOW
            IntakeElement.BLUE -> RearLightStates.HOLDING_BLUE
            IntakeElement.UNKNOWN -> RearLightStates.HOLDING_UNKOWN
            else -> RearLightStates.EMPTY_AND_OFF
        }
    }
    private fun updateTelemetry() {
        val loopTime = loopTimer.elapsedTime
        loopTimer.resetTimer()
        mt.addData("a loop time (ms)", loopTime)

        mt.addData("b path state", pathState.toString())
        mt.addData("c X", follower.pose.x)
        mt.addData("d Y", follower.pose.y)
        mt.addData("e Heading (deg)", Math.toDegrees(follower.pose.heading))
        mt.addData("e1 follower.isBusy ", follower.isBusy)
        mt.addData("e2 armOk() ", armOk())
        mt.addData("e3 seg time ", pathSegmentElapsedTime)

        mt.addData("f Arm State", arm.state)
        mt.addData("g Arm Shoulder Target Angle", arm.targetShoulderAngle)
        mt.addData("h Arm Shoulder Angle", arm.shoulderAngle)
        mt.addData("i Arm Elbow Target", arm.targetElbowAngle)
        mt.addData("j Arm Elbow Angle", arm.elbowAngle)

        mt.update()
    }


}