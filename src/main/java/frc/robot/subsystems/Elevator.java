// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

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

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  // ELEVATOR
  private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.CAN_ELEVATOR_MOTOR, MotorType.kBrushless);
  // private SparkClosedLoopController elevatorController = elevatorMotor.getClosedLoopController();
  private ProfiledPIDController profElevatorController = new ProfiledPIDController(
    0.01, 0, 0.01, // ******UPDATE THIS!!!!!*********
    new TrapezoidProfile.Constraints(2, 1.5));
  private RelativeEncoder elevatorRelativeEncoder = elevatorMotor.getEncoder();
  // private ElevatorFeedforward elevatorFeedforward = new ElevatorFeedforward(0, 0, 0); // *****ALSO UPDATE THIS!!!!******
  // private SparkMax elevatorFollowingMotor = new SparkMax(ElevatorConstants.CAN_FOLLOW_ELEVATOR_MOTOR, MotorType.kBrushless);

  // SHUTTLE
  private SparkMax shuttleMotor = new SparkMax(ElevatorConstants.CAN_SHUTTLE_MOTOR, MotorType.kBrushless);
  private RelativeEncoder shuttleRelativeEncoder = shuttleMotor.getEncoder();
  private ProfiledPIDController profShuttleController = new ProfiledPIDController(
    0.01, 0, 0.01, // UPDATE!!!
    new TrapezoidProfile.Constraints(2, 1.5));
  // private ElevatorFeedforward shuttleFeedforward = new ElevatorFeedforward(0, 0, 0);
  
  // OTHER
  private double elevatorPos = 0; // height from bottom elevtor position to bottom of shuttle slide
  private double shuttlePos = 0; // from bottom of shuttle slide to **TBD**

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
  
    // elevatorConfig.closedLoop.pid(0.01, 0, 0.01);
    // // elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    // elevatorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorPos = elevatorRelativeEncoder.getPosition() / ElevatorConstants.ELEVATOR_TICKS_PER_INCH;
    shuttlePos = shuttleRelativeEncoder.getPosition() / ElevatorConstants.SHUTTLE_TICKS_PER_INCH;
  }

  public double getTotalHeightInches(){
    return elevatorPos + shuttlePos;
  }

  private boolean isAtHeight(double height){
    return (height == getTotalHeightInches());
  }

  private void setElevatorPosition(double pos){
    if (pos >= ElevatorConstants.ELEVATOR_MAX_HEIGHT || pos < 0) return;
    // elevatorController.setReference(height, ControlType.kPosition);
    elevatorMotor.setVoltage(profElevatorController.calculate(elevatorRelativeEncoder.getPosition() + ElevatorConstants.ELEVATOR_FEEDFORWARD));
  }

  private void setShuttlePosition(double pos){
    if (pos >= ElevatorConstants.SHUTTLE_LENGTH || pos < 0) return;
    // elevatorController.setReference(height, ControlType.kPosition);
    shuttleMotor.setVoltage(profShuttleController.calculate(shuttleRelativeEncoder.getPosition() + ElevatorConstants.SHUTTLE_FEEDFORWARD));
  }

  // private void setTotalPosition(double height){
    
  // }

  public Command dynamicElevatorSetPosition(DoubleSupplier height){
    double heightShuttle = (height.getAsDouble()/ElevatorConstants.ELEVATOR_TOTAL_MAX_HEIGHT) * ElevatorConstants.SHUTTLE_LENGTH;
    double heightElevator = (height.getAsDouble()/ElevatorConstants.ELEVATOR_TOTAL_MAX_HEIGHT) * ElevatorConstants.ELEVATOR_MAX_HEIGHT;
    return Commands.parallel(run(() -> setShuttlePosition(heightShuttle)), run(() -> setElevatorPosition(heightElevator)));
  }
}
