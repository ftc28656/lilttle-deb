package org.firstinspires.ftc.teamcode.opmodes.auto

import org.firstinspires.ftc.teamcode.pedroPathing.localization.Pose
import org.firstinspires.ftc.teamcode.pedroPathing.pathGeneration.Point

class AutoConfig(
    @JvmField var maxPowerFraction : Double= 1.0, // 0-1
    @JvmField var armSettleTime : Double= 1.0, // seconds
    @JvmField var armTargetTimeOut : Double= 3.0, // seconds
    @JvmField var bucket : BucketConfig = BucketConfig()
) {
}

class BucketConfig(
    @JvmField var startPose : AutoPose = AutoPose(7.5, 112.5, 270.0),
    @JvmField var scoringPose : AutoPose = AutoPose(16.5, 126.0,315.0),
    @JvmField var lineUpSample1Pose : AutoPose = AutoPose(18.0, 129.0, 355.0),
    @JvmField var pickupSample1Pose : AutoPose = AutoPose(18.0, 126.0, 340.0),
    @JvmField var lineupSample2Pose : AutoPose = AutoPose(18.0, 126.0, 0.0),
    @JvmField var pickupSample2Pose : AutoPose = AutoPose(18.0, 128.0, 15.0),
    @JvmField var lineupSample3Pose : AutoPose = AutoPose(43.0, 110.5, 90.0),
    @JvmField var pickupSample3Pose : AutoPose = AutoPose(43.0, 118.0, 90.0),
    @JvmField var parkPose : AutoPose = AutoPose(60.0, 101.0, 270.0),
    @JvmField var parkControl1 : AutoPoint = AutoPoint(60.0, 124.0),
    @JvmField var parkControl2 : AutoPoint = AutoPoint(60.0, 124.0),
) { }

class AutoPose(
    @JvmField var x : Double = 0.0, // inches
    @JvmField var y : Double = 0.0, // inches
    @JvmField var heading : Double = 0.0, // degrees
){
    fun toPose() : Pose {
        return Pose(x,y,Math.toRadians(heading))
    }
}
class AutoPoint(
@JvmField var x : Double = 0.0, // inches
@JvmField var y : Double = 0.0, // inches
 ) {
    fun toPoint() : Point {
        return Point(x,y)
    }

}