// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Constants;

/** COSNTANTS FOR ALGAE */
public class AlgaeConstants {
    public static final int CAN_ALGAE_PICKUP_MOTOR = 21;
    public static final int CAN_ALGAE_PIVOT_MOTOR = 22;
    public static final int ALGAE_SENSOR_CHANNEL = 2;

    public static final int ARM_FEEDFORWARD_KS = 1;
    public static final int ARM_FEEDFORWARD_KG = 1;
    public static final int ARM_FEEDFORWARD_KV = 1;

    public static final double PIVOT_KP = 0.008;
    public static final double PIVOT_KI = 0;
    public static final double PIVOT_KD = 0.0001;

    public static final double PIVOT_STORE_DEGREES = 0; // Set up zero on encoder with algae arm up
    public static final double PIVOT_DOWN_DEGREES = -20;
    public static final double PIVOT_GROUND_DEGREES = -40;

    public static final double PICKUP_INTAKE_SPEED = -0.5;
    public static final double PICKUP_OUTTAKE_SPEED = 0.5;
}
