// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.objectiveTracker.ButtonBoard;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelecterIONetworkTables;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelectorIO.MoveDirection;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Intake;

import java.io.File;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
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
  private final ObjectiveTracker objectiveTracker;
  private ButtonBoard buttonBoard;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final Joystick buttonBoardController =
      new Joystick(OperatorConstants.BUTTON_BOARD_PORT);
  private final BooleanSupplier[] buttonSupplier = new BooleanSupplier[9];

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
    intake = new Intake();
    algae = new Algae();

    ObjectiveSelecterIONetworkTables objectiveSelecterIOImpl = new ObjectiveSelecterIONetworkTables();
    objectiveTracker = new ObjectiveTracker(objectiveSelecterIOImpl);

    for (int i = 1; i <= 9; i++) {
      final int index = i;
      buttonSupplier[index] = () -> buttonBoardController.getRawButton(index);
    }
    buttonBoard = new ButtonBoard(buttonSupplier);

    SmartDashboard.putNumber("speed", 0.25);
    // Configure the trigger bindings
    configureBindings();

    Shuffleboard.getTab("Debug").addString("Selected Node", objectiveTracker::getObjectiveString);
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
    driverController.a().whileTrue(algae.dynamicAlgaePickup(() -> SmartDashboard.getNumber("Algae Speed", 0.25)));
    
    driverController.povUp().onTrue(objectiveTracker.moveIndex(MoveDirection.UP));
    driverController.povDown().onTrue(objectiveTracker.moveIndex(MoveDirection.DOWN));
    driverController.povRight().onTrue(objectiveTracker.moveIndex(MoveDirection.RIGHT));
    driverController.povLeft().onTrue(objectiveTracker.moveIndex(MoveDirection.LEFT));
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
