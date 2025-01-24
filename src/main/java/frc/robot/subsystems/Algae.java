// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import java.security.PublicKey;
import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.IntakeConstants;

public class Algae extends SubsystemBase {
  private SparkMax algaePickupMotor = new SparkMax(AlgaeConstants.CAN_ALGAE_PICKUP_MOTOR, MotorType.kBrushless);
  private SparkMax algaePivotMotor = new SparkMax(AlgaeConstants.CAN_ALGAE_PIVOT_MOTOR, MotorType.kBrushless);
  private SparkClosedLoopController algaePivotController = algaePivotMotor.getClosedLoopController();
  /** Creates a new Algae. */
  public Algae() {
    SparkMaxConfig algaeConfig = new SparkMaxConfig();
    SparkMaxConfig pivotConfig = new SparkMaxConfig();

    algaeConfig.smartCurrentLimit(20, 20);
    pivotConfig.smartCurrentLimit(20,20);

    algaeConfig.idleMode(IdleMode.kBrake);
    pivotConfig.idleMode(IdleMode.kBrake);

    algaePickupMotor.configure(algaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
    pivotConfig.closedLoop.pid(0.01, 0, 0.01);
    pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    algaePivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setAlgaePivotPosition(double degrees) {
    algaePivotController.setReference(degrees, ControlType.kPosition);
  }

  public Command dynamicAlgaePickup(DoubleSupplier speed){
    return run(() -> algaePickupMotor.set(speed.getAsDouble()));
  }

  public Command dynamicAlgaeSetPivot(DoubleSupplier degrees){
    return run(() -> setAlgaePivotPosition(degrees.getAsDouble()));
  }

  public Command dynamicAlgaeSpeedPivot(DoubleSupplier speed){
    return run(() -> algaePivotMotor.set(speed.getAsDouble()));
  }

  public Command stopAlgaePickup(){
    return run(() -> algaePickupMotor.set(0));
  }

  public Command stopAlgaePivot(){
    return run(() -> algaePivotMotor.set(0));
  }
}
