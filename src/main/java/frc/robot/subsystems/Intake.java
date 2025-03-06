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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  @Logged(name="Coral Intake Motor")
  private final SparkMax intakeMotor = new SparkMax(IntakeConstants.CAN_INTAKE_MOTOR, MotorType.kBrushless);

  DigitalInput coralSensor = new DigitalInput(0);
  
  /** Creates a new IntakeSubsystem. */
  public Intake() {
    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    intakeConfig.smartCurrentLimit(20, 20);

    intakeConfig.idleMode(IdleMode.kCoast);

    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  @Logged(name = "Is coral detected")
  public boolean isCoralDetected() {
    return coralSensor.get();
  }

  private void setIntake(double speed) {
    intakeMotor.set(speed);
  }

  public Command dynamicDriveIntake(DoubleSupplier speed){
    return run(() -> setIntake(-speed.getAsDouble())).finallyDo(() -> setIntake(0));
  }
  
  public Command startStopIntake(DoubleSupplier speed) {
    return startEnd(() -> setIntake(-speed.getAsDouble()), () -> stopIntake());
  }

  public Command intakeCoralWithSensor() {
    return run(() -> setIntake(IntakeConstants.CORAL_INTAKE_SPEED)).until(this::isCoralDetected).finallyDo(this::stopIntake);
  }

  public Command stopIntake(){
    return run(() -> setIntake(0));
  }
  
  @Logged(name = "Is intake limit switch pressed")
  public boolean isIntakeLimitSwitchPressed(){
    return intakeMotor.getForwardLimitSwitch().isPressed();
  }
}