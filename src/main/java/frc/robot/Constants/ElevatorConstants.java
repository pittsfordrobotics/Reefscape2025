package frc.robot.Constants;


public class ElevatorConstants {
    public static final int ELEVATOR_MAX_VELOCITY = 5000;
    public static final int ELEVATOR_MAX_ACCELERATION = 20000;

    public static final int STALL_LIMIT = 40;
    public static final int FREE_LIMIT = 40;

    public static final int CAN_ELEVATOR_MOTOR = 31;
    public static final int CAN_SHUTTLE_MOTOR = 32;

    public static final double ELEVATOR_TICKS_PER_INCH = 20;
    public static final double SHUTTLE_TICKS_PER_INCH = 10;

    public static final double ELEVATOR_MAX_HEIGHT_INCHES = 100;
    public static final double ELEVATOR_MAX_HEIGHT = 170;
    public static final double ELEVATOR_TOTAL_MAX_HEIGHT_INCHES = 120;
    public static final double SHUTTLE_LENGTH_INCHES = 20;
    public static final double SHUTTLE_MAX_HEIGHT = 580;
    public static final double GROUND_TO_ELEVATOR_BOTTOM_INCHES = 10;

    public static final double ELEVATOR_FEEDFORWARD = 0.02;

    public static final double CLOSED_LOOP_RAMP_RATE = 0.15;

    public static final double ELEVATOR_Kp = 0.15;
    public static final double ELEVATOR_Ki = 0;
    public static final double ELEVATOR_Kd = 0.2;

    public static final double SHUTTLE_Kp = 0.05;
    public static final double SHUTTLE_Ki = 0;
    public static final double SHUTTLE_Kd = 0.01;

    public static final double INTAKE_POSITION = 0;
    public static final double L1_POSITION = 15.5;
    public static final double L2_POSITION = 41.5;
    public static final double L3_POSITION = 74.5;
    public static final double L4_POSITION = 125.5;
}
