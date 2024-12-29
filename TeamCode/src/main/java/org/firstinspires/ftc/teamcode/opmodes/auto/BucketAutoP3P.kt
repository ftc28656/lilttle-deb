package org.firstinspires.ftc.teamcode.opmodes.auto

import com.qualcomm.robotcore.eventloop.opmode.Autonomous
import com.qualcomm.robotcore.eventloop.opmode.OpMode
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.ArmPositions
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.ArmSubsystem
import org.firstinspires.ftc.teamcode.pedroPathing.follower.Follower
import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierCurve
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.BezierLine
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.PathChain
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point
import org.firstinspires.ftc.teamcode.pedroPathing.util.Timer

@Autonomous(name = "Bucket Auto Preload+3+Park", group = "Auto")
class BucketAutoP3P : OpMode() {
    private lateinit var follower: Follower
    private var pathTimer = Timer()
    private var opmodeTimer = Timer()
    private var pathState = BucketPathStates.START
    lateinit var arm: ArmSubsystem

    private val startPose = Pose(9.0, 111.0, Math.toRadians(270.0))
    private val scorePose = Pose(14.0, 129.0, Math.toRadians(315.0))
    private val pickupSample1Pose = Pose(37.0, 121.0, Math.toRadians(0.0))
    private val pickupSample2Pose = Pose(43.0, 130.0, Math.toRadians(0.0))
    private val pickupSample3Pose = Pose(49.0, 135.0, Math.toRadians(0.0))
    private val parkPose = Pose(60.0, 98.0, Math.toRadians(90.0))
    private val parkControlPose = Pose(60.0, 98.0, Math.toRadians(90.0))

    private lateinit var scorePreload: PathChain
    private lateinit var park: PathChain
    private lateinit var pickupSample1: PathChain
    private lateinit var pickupSample2: PathChain
    private lateinit var pickupSample3: PathChain
    private lateinit var scoreSample1: PathChain
    private lateinit var scoreSample2: PathChain
    private lateinit var scoreSample3: PathChain

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

        /* This is our pickupSample1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickupSample1 = follower.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(pickupSample1Pose)))
            .setLinearHeadingInterpolation(scorePose.heading, pickupSample1Pose.heading)
            .build()

        /* This is our scoreSample1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSample1 = follower.pathBuilder()
            .addPath(BezierLine(Point(pickupSample1Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(pickupSample1Pose.heading, scorePose.heading)
            .build()

        /* This is our pickupSample2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickupSample2 = follower.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(pickupSample2Pose)))
            .setLinearHeadingInterpolation(scorePose.heading, pickupSample2Pose.heading)
            .build()

        /* This is our scoreSample2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSample2 = follower.pathBuilder()
            .addPath(BezierLine(Point(pickupSample2Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(pickupSample2Pose.heading, scorePose.heading)
            .build()

        /* This is our pickupSample3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        pickupSample3 = follower.pathBuilder()
            .addPath(BezierLine(Point(scorePose), Point(pickupSample3Pose)))
            .setLinearHeadingInterpolation(scorePose.heading, pickupSample3Pose.heading)
            .build()

        /* This is our scoreSample3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scoreSample3 = follower.pathBuilder()
            .addPath(BezierLine(Point(pickupSample3Pose), Point(scorePose)))
            .setLinearHeadingInterpolation(pickupSample3Pose.heading, scorePose.heading)
            .build()

        /* This is our park pathChain. We are using a BezierCurve with 3 points, which is a curved line that is curved based off of the control point */
        park = follower.pathBuilder()
            .addPath(BezierCurve(Point(scorePose), Point(parkControlPose), Point(parkPose)))
            .setLinearHeadingInterpolation(scorePose.heading, parkPose.heading)
            .build()
    }

    /** This function is used to update the state machine for the autonomous.  To extend it, just add more states and transitions.
     * */
    private fun autonomousPathUpdate() {
        when (pathState) {
            BucketPathStates.START ->
                stateTransition(BucketPathStates.SCORING_PRELOAD) {
                    follower.followPath(scorePreload, true)
                }

            BucketPathStates.SCORING_PRELOAD ->
                if (isCloseToTargetPose(scorePose))
                    stateTransition(BucketPathStates.PICKUP_SAMPLE_1) {
                        arm.targetPosition = ArmPositions.SCORE_BUCKET_HIGH
                        follower.followPath(pickupSample1, true)
                    }

            BucketPathStates.PICKUP_SAMPLE_1 ->
                if (isCloseToTargetPose(pickupSample1Pose))
                    stateTransition(BucketPathStates.SCORING_SAMPLE_1) {
                        arm.targetPosition =ArmPositions.SAMPLE_INTAKE
                        follower.followPath(scoreSample1, true)
                    }

            BucketPathStates.SCORING_SAMPLE_1 ->
                if(isCloseToTargetPose(scorePose))
                    stateTransition(BucketPathStates.PICKUP_SAMPLE_2) {
                    arm.targetPosition = ArmPositions.SCORE_BUCKET_HIGH
                    follower.followPath(pickupSample2, true)
                }

            BucketPathStates.PICKUP_SAMPLE_2 ->
                if(isCloseToTargetPose(pickupSample2Pose))
                    stateTransition(BucketPathStates.SCORING_SAMPLE_2) {
                        arm.targetPosition = ArmPositions.SAMPLE_INTAKE
                        follower.followPath(scoreSample2, true)
                    }

            BucketPathStates.SCORING_SAMPLE_2 ->
                if(isCloseToTargetPose(scorePose))
                    stateTransition(BucketPathStates.PICKUP_SAMPLE_3) {
                        arm.targetPosition = ArmPositions.SCORE_BUCKET_HIGH
                        follower.followPath(pickupSample3, true)
                    }

            BucketPathStates.PICKUP_SAMPLE_3 ->
                if(isCloseToTargetPose(pickupSample3Pose))
                    stateTransition(BucketPathStates.SCORING_SAMPLE_3) {
                        arm.targetPosition = ArmPositions.SAMPLE_INTAKE
                        follower.followPath(scoreSample3, true)
                    }

            BucketPathStates.SCORING_SAMPLE_3 ->
                if(isCloseToTargetPose(scorePose))
                    stateTransition(BucketPathStates.PARKING) {
                        arm.targetPosition = ArmPositions.SCORE_BUCKET_HIGH
                        follower.followPath(park, true)
                    }

            BucketPathStates.PARKING ->
                if(isCloseToTargetPose(parkPose))
                    stateTransition(BucketPathStates.PARKING) {
                        arm.targetPosition = ArmPositions.PARK
                    }
        }
    }

    /** This function checks if the robot is close to the target pose, Default tolerance is 1.0 in x and y, and 5 degrees in heading
     * */
    private fun isCloseToTargetPose(
        target: Pose,
        poseTolerance: Pose = Pose(1.0, 1.0, Math.toRadians(5.0))
    ): Boolean {
        return Math.abs(follower.pose.x - target.x) < poseTolerance.x
                && Math.abs(follower.pose.y - target.y) < poseTolerance.y
                && AngleUnit.normalizeRadians(Math.abs(follower.pose.heading - target.heading)) < poseTolerance.heading
    }

    /** This function is used to transition between states in the state machine
     * It will also reset the pathTimer for each state   */
    private fun stateTransition(
        nextState: BucketPathStates,
        action: () -> Unit
    ) {
        action()
        pathState = nextState
        pathTimer.resetTimer()
    }


    override fun init() {
        opmodeTimer.resetTimer()

        follower = Follower(hardwareMap)
        follower.setStartingPose(startPose)

        buildPaths()

        arm = ArmSubsystem(hardwareMap)
        arm.init()
    }

    override fun loop() {
        // These loop the movements of the robot
        follower.update()
        arm.update()
        autonomousPathUpdate()

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState.toString())
        telemetry.addData("x", follower.pose.x)
        telemetry.addData("y", follower.pose.y)
        telemetry.addData("heading", follower.pose.heading)
        telemetry.update()
    }
}