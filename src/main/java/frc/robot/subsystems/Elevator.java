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
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorConstants;

public class Elevator extends SubsystemBase {
  // ELEVATOR
  @Logged(name = "Elevator Motor")
  private SparkMax elevatorMotor = new SparkMax(ElevatorConstants.CAN_ELEVATOR_MOTOR, MotorType.kBrushless);
  private SparkClosedLoopController elevatorController = elevatorMotor.getClosedLoopController();
  private RelativeEncoder elevatorRelativeEncoder = elevatorMotor.getEncoder();
 
  // SHUTTLE
  // @Logged(name = "Shuttle Motor")
  // private SparkMax shuttleMotor = new SparkMax(ElevatorConstants.CAN_SHUTTLE_MOTOR, MotorType.kBrushless);
  // private SparkClosedLoopController shuttleController = shuttleMotor.getClosedLoopController();
  // private RelativeEncoder shuttleRelativeEncoder = shuttleMotor.getEncoder();
  // private ProfiledPIDController profShuttleController = new ProfiledPIDController(
  //   ElevatorConstants.SHUTTLE_Kp, ElevatorConstants.SHUTTLE_Ki, ElevatorConstants.SHUTTLE_Kd, 
  //   new TrapezoidProfile.Constraints(2, 1.5));
  
  private double elevatorPos = 0;  // height from bottom elevtor position to bottom of shuttle slide
  // public double shuttlePos = 0; // from bottom of shuttle slide to **TBD**
  public boolean elevatorIsHomed = false;
  // public boolean shuttleIsHomed = false;

  @Logged(name = "Elevator position inches")
  public double getElevatorPosition() {
    return elevatorPos;
  }
  // @Logged(name = "Shuttle position inches")
  // public double getShuttlePosition(){
  //   return shuttlePos;
  // }
  @Logged(name = "Is elevator homed")
  public boolean getElevatorIsHomed(){
    return elevatorIsHomed;
  }
  // @Logged(name = "Is shuttle homed")
  // public boolean getShuttleIsHomed(){
  //   return shuttleIsHomed;
  // }

  /** Creates a new Elevator. */
  public Elevator() {
    SparkMaxConfig elevatorConfig = new SparkMaxConfig();
    elevatorConfig.smartCurrentLimit(40, 40);
    elevatorConfig.idleMode(IdleMode.kBrake)
    .closedLoopRampRate(0.15);
   
  
    elevatorConfig.closedLoop.maxMotion
      .maxVelocity(5000)
      .maxAcceleration(20000);
    elevatorConfig.closedLoop
      .pid(ElevatorConstants.ELEVATOR_Kp, ElevatorConstants.ELEVATOR_Ki, ElevatorConstants.ELEVATOR_Kd);
    
    elevatorMotor.configure(elevatorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  

    // SparkMaxConfig shuttleConfig = new SparkMaxConfig();
    // shuttleConfig.smartCurrentLimit(20);
    // shuttleConfig.idleMode(IdleMode.kBrake);
   

    // shuttleConfig.closedLoop.maxMotion
    //   .maxVelocity(2)
    //   .maxAcceleration(1.5);
    // shuttleConfig.closedLoop
    //   .pid(ElevatorConstants.SHUTTLE_Kp, ElevatorConstants.SHUTTLE_Ki, ElevatorConstants.SHUTTLE_Kd);

    // shuttleMotor.configure(shuttleConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    elevatorPos = elevatorRelativeEncoder.getPosition() / ElevatorConstants.ELEVATOR_TICKS_PER_INCH;
    // shuttlePos = shuttleRelativeEncoder.getPosition() / ElevatorConstants.SHUTTLE_TICKS_PER_INCH;
  }

  @Logged(name = "Total Height Inches")
  public double getTotalHeightInches(){
    // return elevatorPos + shuttlePos + ElevatorConstants.GROUND_TO_ELEVATOR_BOTTOM_INCHES;
    return elevatorPos + ElevatorConstants.GROUND_TO_ELEVATOR_BOTTOM_INCHES;
  }

  private boolean isAtHeight(double height){
    return (height == getTotalHeightInches());
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

  // public Command homeShuttle(){
  //   return run(() -> shuttleMotor.set(-0.05)).raceWith(Commands.waitUntil(this::isShuttleAtLimit))
  //     .andThen(run(() -> {
  //       shuttleMotor.set(0);
  //       shuttleRelativeEncoder.setPosition(0);
  //       shuttleIsHomed = true;
  //     }));
  // }

  // private boolean isShuttleAtLimit(){
  //   return shuttleMotor.getReverseLimitSwitch().isPressed();
  // }

  /* SETTING POSITION */
  private void setElevatorPosition(double pos){
    /*
     * coral encoder for shuttle (elevator @ bottom): -436
     * coral L2 encoder for elevator (shuttle @ previous pos): 62
     * L3: 109
     * L4: max elevator, shuttle @ -266, robot back ~6"
     */
    elevatorIsHomed = false;
    // if (pos >= ElevatorConstants.ELEVATOR_MAX_HEIGHT || pos < 0) return;
    elevatorController.setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, ElevatorConstants.ELEVATOR_FEEDFORWARD);

    //elevatorMotor.set(profElevatorController.calculate(elevatorRelativeEncoder.getPosition(), pos) + ElevatorConstants.ELEVATOR_FEEDFORWARD);
  }

  // private void setShuttlePosition(double pos){
  //   shuttleIsHomed = false;
  //   // if (pos <= ElevatorConstants.SHUTTLE_MAX_HEIGHT || pos > 0) return;
  //   // shuttleController.setReference(pos, ControlType.kPosition, ClosedLoopSlot.kSlot0, 0); //don't worry about feedforward for the shuttle
  //   //shuttleMotor.set(profShuttleController.calculate(shuttleRelativeEncoder.getPosition(), pos) + ElevatorConstants.SHUTTLE_FEEDFORWARD);
  // }

  public Command dynamicElevatorSetPosition(DoubleSupplier height) {
    return run(() -> {
      double heightRatio = ((height.getAsDouble() - ElevatorConstants.GROUND_TO_ELEVATOR_BOTTOM_INCHES)/ElevatorConstants.ELEVATOR_TOTAL_MAX_HEIGHT_INCHES);
      // double shuttleTargetHeightInches = heightRatio * ElevatorConstants.SHUTTLE_LENGTH_INCHES;
      double elevatorTargetHeightInches = heightRatio * ElevatorConstants.ELEVATOR_MAX_HEIGHT_INCHES;
      // setShuttlePosition(shuttleTargetHeightInches);
      setElevatorPosition(elevatorTargetHeightInches);
    });
  }

  /**
   * Sets the position of the elevator and shuttle, based on an elevator level
   */
  public Command setElevatorLevel(ElevatorLevels level) {
    Command elevatorCommand;
    switch (level) {
      case INTAKE -> {
        // elevatorCommand = run(() -> {setElevatorPosition(0); setShuttlePosition(-436);});
        elevatorCommand = run(() -> setElevatorPosition(0));
      } case L2 -> {
        // elevatorCommand = run(() -> {setElevatorPosition(62); setShuttlePosition(-436);});
        elevatorCommand = run(() -> setElevatorPosition(62));
      } case L3 -> {
        // elevatorCommand = run(() -> {setElevatorPosition(109); setShuttlePosition(-436);});
        elevatorCommand = run(() -> setElevatorPosition(109));
      } case L4 -> {
        // elevatorCommand = run(() -> {
        //   setElevatorPosition(ElevatorConstants.ELEVATOR_MAX_HEIGHT);
        //   setShuttlePosition(-266);
        // });
        elevatorCommand = run(() -> {
            setElevatorPosition(ElevatorConstants.ELEVATOR_MAX_HEIGHT);
          });
      } default -> throw new IllegalArgumentException();
    }
    return elevatorCommand;
  }

  public Command dynamicElevatorSetSpeed(DoubleSupplier speed){
    return run(() -> elevatorMotor.set(speed.getAsDouble())).finallyDo(() -> elevatorMotor.set(ElevatorConstants.ELEVATOR_FEEDFORWARD));
  }

  public Command stopElevator() {
    return run(() -> elevatorMotor.set(0));
  }

  // public Command dynamicShuttleSetSpeed(DoubleSupplier speed){
  //   return run(() -> shuttleMotor.set(speed.getAsDouble())).finallyDo(() -> shuttleMotor.set(0));
  // }

  // public Command stopShuttle() {
  //   return run(() -> shuttleMotor.set(0));
  // }

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

  // @Logged(name = "Shuttle applied output (V)")
  // public double getShuttleAppliedCurrent(){
  //   return shuttleMotor.getAppliedOutput();
  // }
  // @Logged(name = "Shuttle output current (A)")
  // public double getShuttleOutputCurrent(){
  //   return shuttleMotor.getOutputCurrent();
  // }
  // @Logged(name = "Shuttle encoder velocity")
  // public double getShuttleEncoderVelocity(){
  //   return shuttleRelativeEncoder.getVelocity();
  // }
  // @Logged(name = "Shuttle motor temperature (C)")
  // public double getShuttleTemp(){
  //   return shuttleMotor.getMotorTemperature();
  // }

  public enum ElevatorLevels {
    INTAKE, L1, L2, L3, L4;
  }
}
