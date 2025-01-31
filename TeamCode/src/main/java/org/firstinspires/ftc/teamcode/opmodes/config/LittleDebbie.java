package org.firstinspires.ftc.teamcode.opmodes.config;


import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.opmodes.auto.AutoConfig;
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ElbowAngles;
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ElbowConfig;
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeColorLimits;
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeConfig;
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeHueLimits;
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.intake.IntakeServoSpeeds;
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ShoulderAngles;
import org.firstinspires.ftc.teamcode.opmodes.config.subsystems.arm.ShoulderConfig;
import org.firstinspires.ftc.teamcode.opmodes.teleop.DriveConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.util.CustomPIDFCoefficients;

/** Everything that we want to store globally, for example positions of servos, motors, etc. goes in here. **/

@Config
public class LittleDebbie {

    /**
     * Drive Parameters
     * **/
    public static DriveConfig drive = new DriveConfig();

    /**
     * Intake Parameters
     * **/
    public static IntakeConfig intake = new IntakeConfig();

    /**
     * Shoulder Parameters
     */
    public static ShoulderConfig shoulder = new ShoulderConfig();

    /**
     * Elbow Config
     */
    public static ElbowConfig elbow = new ElbowConfig();

    public  static AutoConfig auto = new AutoConfig();

}
