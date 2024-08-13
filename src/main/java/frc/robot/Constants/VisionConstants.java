// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** Vision CONSTANTS! Do not create limelight or other VisionIO objects here, they need to be assigned in RobotContainer.java */
public final class VisionConstants {

    //Setting up names of limelights
    public final static String LIMELIGHT1_NAME = "limelight-one";
    public final static String LIMELIGHT2_NAME = "limelight-two";

    public static final double FIELD_BORDER_MARGIN = 0.5;
    public static final double Z_MARGIN = 0.75;
    public static final double XY_STD_DEV_COEF = 0.2;
    //Don't even bother using vision for heading measurement, Pigeon2 is good enough
    public static final double THETA_STD_DEV_COEF = 1;
    public static final double TARGET_LOG_SECONDS = 0.1;
}
