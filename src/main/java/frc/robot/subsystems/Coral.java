// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CoralConstants;
import frc.robot.logging.SparkMaxLogger;

public class Coral extends SubsystemBase {
  @Logged(name = "Coral Output Motor")
  private SparkMax coralMotor = new SparkMax(CoralConstants.CAN_CORAL_MOTOR, MotorType.kBrushless);


  /** Creates a new Coral. */
  public Coral() {
    SparkMaxConfig coralConfig = new SparkMaxConfig();
    coralConfig.smartCurrentLimit(20, 20);
    coralConfig.idleMode(IdleMode.kCoast);
    coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void setCoral(double speed) {
    coralMotor.set(speed);
  }

  public Command placeCoral(){
    return run(() -> setCoral(CoralConstants.CORAL_SPEED));
  }

  public Command dynamicDriveCoral(DoubleSupplier speed) {
    return run(() -> setCoral(speed.getAsDouble())).finallyDo(() -> setCoral(0));
  }

  public Command stopCoral() {
    return run(() -> setCoral(0));
  }
}
