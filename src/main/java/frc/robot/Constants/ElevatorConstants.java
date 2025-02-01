package frc.robot.Constants;

public class ElevatorConstants {
    public static final int CAN_ELEVATOR_MOTOR = 31;
    public static final int CAN_SHUTTLE_MOTOR = 32;

    public static final double ELEVATOR_TICKS_PER_INCH = 20;
    public static final double SHUTTLE_TICKS_PER_INCH = 10;

    public static final double ELEVATOR_MAX_HEIGHT = 100; // need to change + decide units
    public static final double SHUTTLE_LENGTH = 20; // ^^^^^^

    public static final double ELEVATOR_FEEDFORWARD = 0.05;
    public static final double SHUTTLE_FEEDFORWARD = 0.05;
}
