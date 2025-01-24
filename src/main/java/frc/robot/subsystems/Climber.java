// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {
  private SparkMax climbMotor1 = new SparkMax(ClimberConstants.CAN_CLIMBER_1, MotorType.kBrushless);
  private SparkMax climbMotor2 = new SparkMax(ClimberConstants.CAN_CLIMBER_2, MotorType.kBrushless);
  private SparkClosedLoopController climbClosedLoopController = climbMotor1.getClosedLoopController();
  /** Creates a new Climber. */
  public Climber() {
    SparkMaxConfig climbConfig1 = new SparkMaxConfig();
    SparkMaxConfig climbConfig2 = new SparkMaxConfig();

    climbConfig1.smartCurrentLimit(20, 20);
    climbConfig2.smartCurrentLimit(20, 20);
    
    climbConfig1.idleMode(IdleMode.kBrake);
    climbConfig2.idleMode(IdleMode.kBrake);

    climbConfig1.closedLoop.pid(0.1, 0, 0.1);

    climbConfig2.follow(climbMotor1);
    climbConfig2.inverted(true);

    climbMotor1.configure(climbConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    climbMotor2.configure(climbConfig2, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setClimbSpeed(double speed) {
    climbMotor1.set(speed);
  }

  public Command dynamicDriveClimb(DoubleSupplier speed){
    return run(() -> setClimbSpeed(speed.getAsDouble()));
  }

  private void setClimbPosition(double degrees) {
    climbClosedLoopController.setReference(degrees, ControlType.kPosition);
  }

  public Command climbToPosition(DoubleSupplier degrees){
    return run(() -> setClimbPosition(degrees.getAsDouble()));
  }

  public Command stopClimb(){
    return run(() -> setClimbSpeed(0));
  }
}