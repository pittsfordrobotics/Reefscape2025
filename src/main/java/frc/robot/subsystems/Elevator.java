// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.LimitSwitchConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  // ELEVATOR
  @Logged(name = "Elevator Motor")
  private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.CAN_ELEVATOR_MOTOR, MotorType.kBrushless);
  private SparkClosedLoopController elevatorController = elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorRelativeEncoder = elevatorMotor.getEncoder();
  
  private double elevatorPos = 0;  // height from bottom elevtor position to bottom of shuttle slide
  public boolean elevatorIsHomed = false;
  private int encoderOffset = 0;

  @Logged(name = "Elevator position inches")
  public double getElevatorPosition() {
    return elevatorPos;
  }
  
  @Logged(name = "Is elevator homed")
  public boolean getElevatorIsHomed(){
    return elevatorIsHomed;
  }

  @Logged(name = "Encoder offset")
  public int getEncoderOffset(){
    return encoderOffset;
  }

  /** Creates a new Elevator. */
  public Elevator() {
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig.smartCurrentLimit(ElevatorConstants.STALL_LIMIT, ElevatorConstants.FREE_LIMIT);
    elevatorConfig.idleMode(IdleMode.kBrake)
    .closedLoopRampRate(ElevatorConstants.CLOSED_LOOP_RAMP_RATE);
    elevatorConfig.inverted(true);
   
  
    elevatorConfig.closedLoop.maxMotion
      .maxVelocity(ElevatorConstants.ELEVATOR_MAX_VELOCITY)
      .maxAcceleration(ElevatorConstants.ELEVATOR_MAX_ACCELERATION);
    elevatorConfig.closedLoop
      .pid(ElevatorConstants.ELEVATOR_Kp, ElevatorConstants.ELEVATOR_Ki, ElevatorConstants.ELEVATOR_Kd);

    LimitSwitchConfig elevatorLimitSwitchConfig = new LimitSwitchConfig();
    elevatorLimitSwitchConfig.forwardLimitSwitchEnabled(true);
    elevatorLimitSwitchConfig.reverseLimitSwitchEnabled(true);
    elevatorConfig.apply(elevatorLimitSwitchConfig);

    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorPos = elevatorRelativeEncoder.getPosition();
  }

  @Logged(name = "Total Height Inches")
  public double getTotalHeightInches(){
    return elevatorPos + ElevatorConstants.GROUND_TO_ELEVATOR_BOTTOM_INCHES;
  }

  /* HOMING */

  public Command homeElevator(){
    return run(() -> elevatorMotor.set(-0.05)).raceWith(Commands.waitUntil(this::isElevatorAtLimit))
      .andThen(run(() -> {
        elevatorMotor.set(0);
        elevatorRelativeEncoder.setPosition(0);
        elevatorIsHomed = true;
      }));
  }

  private boolean isElevatorAtLimit(){
    return elevatorMotor.getReverseLimitSwitch().isPressed();
  }

  /* SETTING POSITION */
  private void setElevatorPosition(double pos){
    /*
     * coral encoder for shuttle (elevator @ bottom): -436
     * coral L2 encoder for elevator (shuttle @ previous pos): 62
     * L3: 109
     * L4: max elevator, shuttle @ -266, robot back ~6"
     */
    elevatorIsHomed = false;
    elevatorController.setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.ELEVATOR_FEEDFORWARD);
  }

  public Command dynamicElevatorSetPosition(DoubleSupplier height) {
    return run(() -> {
      double heightRatio = ((height.getAsDouble() - ElevatorConstants.GROUND_TO_ELEVATOR_BOTTOM_INCHES)/ElevatorConstants.ELEVATOR_TOTAL_MAX_HEIGHT_INCHES);
      double elevatorTargetHeightInches = heightRatio * ElevatorConstants.ELEVATOR_MAX_HEIGHT_INCHES;
      setElevatorPosition(elevatorTargetHeightInches);
    });
  }

  /**
   * Sets the position of the elevator and shuttle, based on an elevator level
   */
  public Command setElevatorLevel(ElevatorLevels level) {
    System.out.println("Setting elevator level to " + level);
    Command elevatorCommand;
    switch (level) {
      case INTAKE -> {
        elevatorCommand = runOnce(() -> setElevatorPosition(ElevatorConstants.INTAKE_POSITION + encoderOffset));
      } case L2 -> {
        elevatorCommand = runOnce(() -> setElevatorPosition(ElevatorConstants.L2_POSITION + encoderOffset));
      } case L3 -> {
        elevatorCommand = runOnce(() -> setElevatorPosition(ElevatorConstants.L3_POSITION + encoderOffset));
      } case L4 -> {
        elevatorCommand = runOnce(() -> {
            setElevatorPosition(ElevatorConstants.L4_POSITION + encoderOffset);
          });
      } default -> throw new IllegalArgumentException();
    }
    return elevatorCommand;
  }

  public Command dynamicElevatorLevel(Supplier<ElevatorLevels> levelSupplier) {
    return run(() -> {
      ElevatorLevels level = levelSupplier.get();
      double elevatorHeight = switch(level) {
        case ZERO -> 0;
        case INTAKE -> ElevatorConstants.INTAKE_POSITION + encoderOffset;
        case L1 -> ElevatorConstants.L1_POSITION + encoderOffset;
        case L2 -> ElevatorConstants.L2_POSITION + encoderOffset;
        case L3 -> ElevatorConstants.L3_POSITION + encoderOffset;
        case L4 -> ElevatorConstants.L4_POSITION + encoderOffset;
        default -> throw new IllegalArgumentException();
      };
      setElevatorPosition(elevatorHeight);
    });
  }
  
  public boolean isAtLevel(Supplier<ElevatorLevels> level) {
    if(Robot.isSimulation()) {
      return true;
    }
    switch (level.get()) {
      case INTAKE -> {
        return Math.abs(ElevatorConstants.INTAKE_POSITION + encoderOffset - elevatorPos) < 2;
      } case L2 -> {
        return Math.abs(ElevatorConstants.L2_POSITION + encoderOffset - elevatorPos) < 2;
      } case L3 -> {
        return Math.abs(ElevatorConstants.L3_POSITION + encoderOffset - elevatorPos) < 2;
      } case L4 -> {
        return Math.abs(ElevatorConstants.L4_POSITION + encoderOffset - elevatorPos) < 2;
      } default -> throw new IllegalArgumentException();
    }
  }

  public Command dynamicElevatorSetSpeed(DoubleSupplier speed){
    return run(() -> elevatorMotor.set(speed.getAsDouble())).finallyDo(() -> elevatorMotor.set(ElevatorConstants.ELEVATOR_FEEDFORWARD));
  }

  public Command stopElevator() {
    return run(() -> elevatorMotor.set(0));
  }

  public enum ElevatorLevels {
    ZERO, INTAKE, L1, L2, L3, L4;
  }

  public void increaseEncoderOffset(int offset){
    encoderOffset += offset;
  }

  public void zeroElevatorOffset(){
    encoderOffset = 0;
  }
}
