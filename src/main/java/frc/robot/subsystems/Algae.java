// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;

public class Algae extends SubsystemBase {
  @Logged(name = "Algae Pickup Motor")
  private SparkMax algaePickupMotor = new SparkMax(AlgaeConstants.CAN_ALGAE_PICKUP_MOTOR, MotorType.kBrushless);
  @Logged(name = "Algae Pivot Motor")
  private SparkMax algaePivotMotor = new SparkMax(AlgaeConstants.CAN_ALGAE_PIVOT_MOTOR, MotorType.kBrushless);

  // @Logged(name = "Algae Sensor")
  // DigitalInput algaeSensor = new DigitalInput(AlgaeConstants.ALGAE_SENSOR_CHANNEL);

  private SparkAbsoluteEncoder pivotEncoder;

  private ProfiledPIDController algaePivotProfiledPIDController = new ProfiledPIDController(
    AlgaeConstants.PROFILED_PID_KP, AlgaeConstants.PROFILED_PID_KI, AlgaeConstants.PROFILED_PID_KD, new TrapezoidProfile.Constraints(
      AlgaeConstants.MAX_VELOCITY, AlgaeConstants.MAX_ACCELERATION));
  
  private ArmFeedforward algaePivotFeedforward = new ArmFeedforward(
    AlgaeConstants.ARM_FEEDFORWARD_KS, 
    AlgaeConstants.ARM_FEEDFORWARD_KG, 
    AlgaeConstants.ARM_FEEDFORWARD_KV);
  
  /** Creates a new Algae. */
  public Algae() {
    SparkMaxConfig algaeConfig = new SparkMaxConfig();
    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    algaeConfig.smartCurrentLimit(40, 40);
    pivotConfig.smartCurrentLimit(40,40);

    algaeConfig.idleMode(IdleMode.kBrake);
    pivotConfig.idleMode(IdleMode.kBrake);

    algaePickupMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    
    pivotConfig.absoluteEncoder.positionConversionFactor(1);

    algaePivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    pivotEncoder = algaePivotMotor.getAbsoluteEncoder();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // @Logged(name = "IsAlgaeDetected")
  // public boolean isAlgaeDetected() {
  //   return algaeSensor.get();
  // }

  private void setAlgaePivotPosition(double degrees) {
    algaePivotMotor.set(
      algaePivotProfiledPIDController.calculate(pivotEncoder.getPosition(), degrees) 
    + algaePivotFeedforward.calculate(pivotEncoder.getPosition(), pivotEncoder.getVelocity()));
  }

  public Command dynamicAlgaePickup(DoubleSupplier speed){
    return run(() -> algaePickupMotor.set(speed.getAsDouble())).finallyDo(() -> algaePickupMotor.set(0));
  }

  public Command dynamicAlgaeSetPivot(DoubleSupplier degrees){
    return run(() -> setAlgaePivotPosition(degrees.getAsDouble()));
  }

  public Command dynamicAlgaeSpeedPivot(DoubleSupplier speed){
    return run(() -> algaePivotMotor.set(-speed.getAsDouble())).finallyDo(() -> algaePivotMotor.set(0));
  }

  public Command startStopDriveAlgae(DoubleSupplier speed) {
    return startEnd(
      () -> algaePickupMotor.set(speed.getAsDouble()),
       () -> algaePickupMotor.set(0));
  }

  public Command startStopAlgaePivot(DoubleSupplier degrees) {
    return startEnd(
      () -> setAlgaePivotPosition(degrees.getAsDouble()),
       () -> algaePivotMotor.set(0));
  }

  public Command dualAlgaeIntake(DoubleSupplier degrees, DoubleSupplier speed){
    return dynamicAlgaeSetPivot(degrees).andThen(startStopDriveAlgae(speed));
  }
  
}
