package frc.robot.Constants;

public class ElevatorConstants {
    public static final int CAN_ELEVATOR_MOTOR = 31;
    public static final int CAN_SHUTTLE_MOTOR = 32;

    public static final double ELEVATOR_TICKS_PER_INCH = 20;
    public static final double SHUTTLE_TICKS_PER_INCH = 10;

    public static final double ELEVATOR_MAX_HEIGHT_INCHES = 100; // need to change + decide units
    public static final double ELEVATOR_TOTAL_MAX_HEIGHT_INCHES = 120;
    public static final double SHUTTLE_LENGTH_INCHES = 20; // ^^^^^^
    public static final double GROUND_TO_ELEVATOR_BOTTOM_INCHES = 10;

    public static final double ELEVATOR_FEEDFORWARD = 0.05;
    public static final double SHUTTLE_FEEDFORWARD = 0.05;

    public static final double ELEVATOR_Kp = 0.01;
    public static final double ELEVATOR_Ki = 0;
    public static final double ELEVATOR_Kd = 0.01;

    public static final double SHUTTLE_Kp = 0.01;
    public static final double SHUTTLE_Ki = 0;
    public static final double SHUTTLE_Kd = 0.01;
}
