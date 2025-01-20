// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final Swerve swerve;
  private final Intake intake;
  private final Algae algae;
  private final Climber climber;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    //swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
    intake = new Intake();
    algae = new Algae();
    climber = new Climber();

    SmartDashboard.putNumber("Intake Speed", -0.25);
    SmartDashboard.putNumber("Algae Speed", 0.25);
    SmartDashboard.putNumber("Algae Pivot Speed", 0.25);
    SmartDashboard.putNumber("Algae Angle 1", 0);
    
    SmartDashboard.putNumber("Climb Speed", 0.25);

    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //Drive Intake:
    driverController.b().whileTrue(intake.dynamicDriveIntake(
      () -> SmartDashboard.getNumber("Intake Speed", -0.25)))
      .onFalse(intake.stopIntake());

      driverController.y().whileTrue(intake.dynamicDriveIntake(
        () -> -1 * SmartDashboard.getNumber("Intake Speed", -0.25)))
        .onFalse(intake.stopIntake());
    
    //Pivot Algae arm:
    //Pos 1
     driverController.leftTrigger().onTrue(algae.dynamicAlgaeSetPivot(
      () -> SmartDashboard.getNumber("Algae Angle 1", 0)));
    // //Pos 2
    // driverController.rightBumper().onTrue(algae.dynamicAlgaeSetPivot(
    //   () -> SmartDashboard.getNumber("Algae Angle 2", 0))); 
    
    //Rotate Algae arm:
    driverController.rightTrigger().whileTrue(algae.dynamicAlgaeSpeedPivot(
      () -> SmartDashboard.getNumber("Algae Pivot Speed", 0.25)))
      .onFalse(algae.stopAlgaePivot());
      
    driverController.rightBumper().whileTrue(algae.dynamicAlgaeSpeedPivot(
      () -> -1 * SmartDashboard.getNumber("Algae Pivot Speed", 0.25)))
      .onFalse(algae.stopAlgaePivot());

    //Drive Algae pickup:
    driverController.a().whileTrue(algae.dynamicAlgaePickup(
      () -> SmartDashboard.getNumber("Algae Speed", 0.25)))
      .onFalse(algae.stopAlgaePickup());
    
    driverController.x().whileTrue(algae.dynamicAlgaePickup(
      () -> -1 * SmartDashboard.getNumber("Algae Speed", 0.25)))
      .onFalse(algae.stopAlgaePickup());

    //Elevator Stuff:
    //

    //Drive Climber:
    driverController.leftTrigger().whileTrue(climber.dynamicDriveClimb(
      () -> SmartDashboard.getNumber(("Climb Speed"), 0.25)))
      .onFalse(climber.stopClimb());
    
    driverController.leftBumper().whileTrue(climber.dynamicDriveClimb(
    () -> -1 * SmartDashboard.getNumber("Climb Speed", 0.25)))
    .onFalse(climber.stopClimb());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous

    return new Command() {};
  }
}
