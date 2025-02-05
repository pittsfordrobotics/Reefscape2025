// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  // ELEVATOR
  private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.CAN_ELEVATOR_MOTOR, MotorType.kBrushless);
  private ProfiledPIDController profElevatorController = new ProfiledPIDController(
    ElevatorConstants.ELEVATOR_Kp, ElevatorConstants.ELEVATOR_Ki, ElevatorConstants.ELEVATOR_Kd,
    new TrapezoidProfile.Constraints(2, 1.5));
  private RelativeEncoder elevatorRelativeEncoder = elevatorMotor.getEncoder();
  // private SparkMax elevatorFollowingMotor = new SparkMax(ElevatorConstants.CAN_FOLLOW_ELEVATOR_MOTOR, MotorType.kBrushless);

  // SHUTTLE
  private SparkMax shuttleMotor = new SparkMax(ElevatorConstants.CAN_SHUTTLE_MOTOR, MotorType.kBrushless);
  private RelativeEncoder shuttleRelativeEncoder = shuttleMotor.getEncoder();
  private ProfiledPIDController profShuttleController = new ProfiledPIDController(
    ElevatorConstants.SHUTTLE_Kp, ElevatorConstants.SHUTTLE_Ki, ElevatorConstants.SHUTTLE_Kd, 
    new TrapezoidProfile.Constraints(2, 1.5));
  
  // OTHER
  @Logged(name = "Elevator Position Inches")
  public double elevatorPos = 0; // height from bottom elevtor position to bottom of shuttle slide
  @Logged(name = "Shuttle Posiiton Inches")
  public double shuttlePos = 0; // from bottom of shuttle slide to **TBD**
  @Logged(name = "Elevator Homed")
  public boolean elevatorIsHomed = false;
  @Logged(name = "Shuttle Homed")
  public boolean shuttleIsHomed = false;

  /** Creates a new Elevator. */
  public Elevator() {
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig.smartCurrentLimit(40, 40);
    elevatorConfig.idleMode(IdleMode.kBrake);
    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    SparkMaxConfig shuttleConfig = new SparkMaxConfig();
    shuttleConfig.smartCurrentLimit(20, 20);
    shuttleConfig.idleMode(IdleMode.kBrake);
    shuttleMotor.configure(shuttleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // SparkMaxConfig elevatorFollowingMotorConfig = new SparkMaxConfig();
    // elevatorFollowingMotorConfig.smartCurrentLimit(20, 20);
    // elevatorFollowingMotorConfig.idleMode(IdleMode.kBrake);
    // elevatorFollowingMotorConfig.follow(elevatorMotor, false);
    // elevatorFollowingMotor.configure(elevatorFollowingMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorPos = elevatorRelativeEncoder.getPosition() / ElevatorConstants.ELEVATOR_TICKS_PER_INCH;
    shuttlePos = shuttleRelativeEncoder.getPosition() / ElevatorConstants.SHUTTLE_TICKS_PER_INCH;
  }

  @Logged(name = "Total Height Inches")
  public double getTotalHeightInches(){
    return elevatorPos + shuttlePos + ElevatorConstants.GROUND_TO_ELEVATOR_BOTTOM_INCHES;
  }

  private boolean isAtHeight(double height){
    return (height == getTotalHeightInches());
  }

  /* HOMING */
  private void afterElevatorHomed(){
    elevatorMotor.set(0);
    elevatorRelativeEncoder.setPosition(0);
    elevatorIsHomed = true;
  }
  public Command homeElevator(){
    return run(() -> elevatorMotor.set(-0.05)).raceWith(Commands.waitUntil(isHomedLimitE())).andThen(run(() -> afterElevatorHomed()));
  }
  private BooleanSupplier isHomedLimitE(){
    return (() -> elevatorMotor.getReverseLimitSwitch().isPressed());
  }

  private void afterShuttleHomed(){
    shuttleMotor.set(0);
    shuttleRelativeEncoder.setPosition(0);
    shuttleIsHomed = true;
  }
  public Command homeShuttle(){
    return run(() -> shuttleMotor.set(-0.05)).raceWith(Commands.waitUntil(isHomedLimitS())).andThen(run(() -> afterShuttleHomed()));
  }
  private BooleanSupplier isHomedLimitS(){
    return (() -> shuttleMotor.getReverseLimitSwitch().isPressed());
  }

  /* SETTING POSITION */
  private void setElevatorPosition(double pos){
    if (pos >= ElevatorConstants.ELEVATOR_MAX_HEIGHT_INCHES || pos < 0) return;
    pos /= ElevatorConstants.ELEVATOR_TICKS_PER_INCH;
    // elevatorController.setReference(height, ControlType.kPosition);
    elevatorMotor.set(profElevatorController.calculate(elevatorRelativeEncoder.getPosition(), pos) + ElevatorConstants.ELEVATOR_FEEDFORWARD);
  }

  private void setShuttlePosition(double pos){
    if (pos >= ElevatorConstants.SHUTTLE_LENGTH_INCHES || pos < 0) return;
    pos /= ElevatorConstants.SHUTTLE_TICKS_PER_INCH;
    // elevatorController.setReference(height, ControlType.kPosition);
    shuttleMotor.set(profShuttleController.calculate(shuttleRelativeEncoder.getPosition(), pos) + ElevatorConstants.SHUTTLE_FEEDFORWARD);
  }

  public Command dynamicElevatorSetPosition(DoubleSupplier height){
    double heightShuttle = ((height.getAsDouble() - ElevatorConstants.GROUND_TO_ELEVATOR_BOTTOM_INCHES)/ElevatorConstants.ELEVATOR_TOTAL_MAX_HEIGHT_INCHES) 
      * ElevatorConstants.SHUTTLE_LENGTH_INCHES;
    double heightElevator = ((height.getAsDouble() - ElevatorConstants.GROUND_TO_ELEVATOR_BOTTOM_INCHES)/ElevatorConstants.ELEVATOR_TOTAL_MAX_HEIGHT_INCHES) 
      * ElevatorConstants.ELEVATOR_MAX_HEIGHT_INCHES;
    return Commands.parallel(run(() -> setShuttlePosition(heightShuttle)), run(() -> setElevatorPosition(heightElevator)));
  }

  /* TELEMETRY */
  @Logged(name = "Elevator applied output (V)")
  public double getElevatorAppliedCurrent(){
    return elevatorMotor.getAppliedOutput();
  }
  @Logged(name = "Elevator output current (A)")
  public double getElevatorOutputCurrent(){
    return elevatorMotor.getOutputCurrent();
  }
  @Logged(name = "Elevator encoder velocity")
  public double getElevatorEncoderVelocity(){
    return elevatorRelativeEncoder.getVelocity();
  }
  @Logged(name = "Elevator motor temperature (C)")
  public double getElevatorTemp(){
    return elevatorMotor.getMotorTemperature();
  }

  @Logged(name = "Shuttle applied output (V)")
  public double getShuttleAppliedCurrent(){
    return shuttleMotor.getAppliedOutput();
  }
  @Logged(name = "Shuttle output current (A)")
  public double getShuttleOutputCurrent(){
    return shuttleMotor.getOutputCurrent();
  }
  @Logged(name = "Shuttle encoder velocity")
  public double getShuttleEncoderVelocity(){
    return shuttleRelativeEncoder.getVelocity();
  }
  @Logged(name = "Shuttle motor temperature (C)")
  public double getShuttleTemp(){
    return shuttleMotor.getMotorTemperature();
  }
}
