// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Intake;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
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
  private final Swerve swerve;
  private final Intake intake;
  private final Algae algae;
  private final Coral coral;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
    intake = new Intake();
    algae = new Algae();
    coral = new Coral();


    Command enhancedHeadingSteeringCommand = swerve.enhancedHeadingDriveCommand(
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightY(),
        () -> -driverController.getRightX(),
        driverController::getLeftTriggerAxis,
        driverController::getRightTriggerAxis);
    swerve.setDefaultCommand(enhancedHeadingSteeringCommand);

    SmartDashboard.putNumber("speed", 0.25);
    Shuffleboard.getTab("Config").add("Zero swerve offsets", swerve.runOnce(() -> swerve.setSwerveOffsets()).ignoringDisable(true));
    Shuffleboard.getTab("Config").add("Set offsets to 0", swerve.runOnce(() -> swerve.zeroSwerveOffsets()).ignoringDisable(true));
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
      () -> SmartDashboard.getNumber("Intake Speed", 0.25)));
    
    //Pivot Algae arm:
    //Pos 1
    driverController.rightTrigger().onTrue(algae.dynamicAlgaePivot(
      () -> SmartDashboard.getNumber("Algae Angle 1", 0)));
    //Pos 2
    driverController.rightBumper().onTrue(algae.dynamicAlgaePivot(
      () -> SmartDashboard.getNumber("Algae Angle 2", 0)));
    
    //Drive Algae pickup:
    driverController.a().whileTrue(algae.dynamicAlgaePickup(
        () -> SmartDashboard.getNumber("Algae Speed", 0.25)));

    //Drive Coral output:
    driverController.leftTrigger().whileTrue(coral.dynamicDriveCoral(
      () -> SmartDashboard.getNumber("Coral Speed", 0.25)))
        .onFalse(coral.stopCoral());

    //Drive Swerve forward and backward:
    driverController.povUp().whileTrue(swerve.driveForward(0.2));
    driverController.povDown().whileTrue(swerve.driveForward(-0.2));


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
