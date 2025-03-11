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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.CoralConstants;

public class Coral extends SubsystemBase {
  @Logged(name = "Coral Output Motor")
  private SparkMax coralMotor = new SparkMax(CoralConstants.CAN_CORAL_MOTOR, MotorType.kBrushless);
  private DigitalInput coralSensor = new DigitalInput(3);

  /** Creates a new Coral. */
  public Coral() {
    SparkMaxConfig coralConfig = new SparkMaxConfig();
    coralConfig.smartCurrentLimit(20, 20);
    coralConfig.idleMode(IdleMode.kBrake);
    coralMotor.configure(coralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Logged(name = "Coral Detected")
  public boolean isCoralDetected() {
    if (Robot.isSimulation()) {
      return true;
    }
    return !coralSensor.get();
  }

  private void setCoral(double speed) {
    coralMotor.set(speed);
    System.out.println("Dropping Coral!");
    System.out.println(speed);
  }

  public Command intakeCoral() {
    return runOnce(() -> setCoral(CoralConstants.CORAL_INTAKE_SPEED));
  }

  public Command placeCoral() {
    return run(() -> setCoral(CoralConstants.CORAL_SPEED))
        .raceWith(Commands.waitSeconds(0.5))
        .andThen(() -> setCoral(0));
  }

  public Command dynamicDriveCoral(DoubleSupplier speed) {
    return run(() -> setCoral(-speed.getAsDouble())).finallyDo(() -> setCoral(0));
  }

  public Command stopCoral() {
    return runOnce(() -> setCoral(0));
  }

  @Logged(name = "Is coral limit switch pressed")
  public boolean isCoralLimitSwitchPressed() {
    return coralMotor.getForwardLimitSwitch().isPressed();
  }
}
