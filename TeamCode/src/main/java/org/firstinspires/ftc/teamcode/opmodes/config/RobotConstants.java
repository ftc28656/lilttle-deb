package org.firstinspires.ftc.teamcode.opmodes.config;


import com.acmerobotics.dashboard.config.Config;

/** Everything that we want to store globally, for example positions of servos, motors, etc. goes in here. **/

@Config
public class RobotConstants {

    public static double intakeServoSpeed = 1.0;
    public static double outtakeServoSpeed = 0.0;
    public static double stopIntakeServoSpeed = 0.5;


    // Angles here are in degrees for easy readability
    public static double elbowParkAngle = 0.0;
    public static double shoulderParkAngle = 0.0;

    public static double elbowSampleIntakeAngle = 0.0;
    public static double shoulderSampleIntakeAngle = 0.0;

    public static double elbowSpecimenIntakeAngle = 0.0;
    public static double shoulderSpecimenIntakeAngle = 0.0;

    public static double elbowScoreBucketHighAngle = 0.0;
    public static double shoulderScoreBucketHighAngle = 0.0;

    public static double elbowScoreBucketMediumAngle = 0.0;
    public static double shoulderScoreBucketMediumAngle = 0.0;

    public static double elbowScoreBucketLowAngle = 0.0;
    public static double shoulderScoreBucketLowAngle = 0.0;

    public static double elbowScoreChamberLowAngle = 0.0;
    public static double shoulderScoreChamberLowAngle = 0.0;

    public static double elbowScoreChamberHighAngle = 0.0;
    public static double shoulderScoreChamberHighAngle = 0.0;

    public static double elbowStartAngle = 0.0;
    public static double shoulderStartAngle = 0.0;

}
