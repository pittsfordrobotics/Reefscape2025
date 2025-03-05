// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Intake;

import frc.robot.subsystems.objectiveTracker.ObjectiveSelecterIONetworkTables;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelectorIO.MoveDirection;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //private final Swerve swerve;
  @Logged(name = "Coral Intake Subsystem")
  private final Intake intake;

  @Logged(name = "Algae Subsystem")
  private final Algae algae;
  private final SendableChooser<Command> autoChooser;
  private final ObjectiveTracker objectiveTracker;

  @Logged(name = "Climber Subsystem")
  private final Climber climber;

  @Logged(name = "Elevator Subsystem")
  private final Elevator elevator;

  @Logged(name = "Coral Output Subsystem")
  private final Coral coral;
  
  @Logged(name = "Swerve Subsystem")
  private final Swerve swerve;
  private final Vision vision;

  @Logged(name = "PDH")
  private final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.DRIVER_CONTROLLER_PORT);
  private final CommandXboxController operatorController =
      new CommandXboxController(OperatorConstants.OPERATOR_CONTROLLER_PORT);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    //swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve/maxSwerve"));
    intake = new Intake();
    algae = new Algae();
    climber = new Climber();
    elevator = new Elevator();
    coral = new Coral();

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
    vision = new Vision(
      VisionConstants.LIMELIGHT_LEFT,
      VisionConstants.LIMELIGHT_RIGHT,
      VisionConstants.LIMELIGHT_BACK,
      swerve::getGyroAngle,
      swerve::getAngularVelocityRad_Sec,
      swerve::addVisionData);

    SmartDashboard.putNumber("Algae Speed", 0.25);

    SmartDashboard.putNumber("Algae Pivot Speed", 0.25);
    SmartDashboard.putNumber("Algae Up Angle", 0);
    SmartDashboard.putNumber("Algae Down Angle", 0);
    
    SmartDashboard.putNumber("Climb Speed", 0.25);
    SmartDashboard.putNumber("Climb Default Angle", 0);
    SmartDashboard.putNumber("Climb Active Angle", 0);
    

    ObjectiveSelecterIONetworkTables objectiveSelecterIOImpl = new ObjectiveSelecterIONetworkTables();
    objectiveTracker = new ObjectiveTracker(objectiveSelecterIOImpl);
    objectiveTracker.setDefaultCommand(objectiveTracker.updateReefSide(swerve::getPose));

    Command headingSteeringCommand = swerve.headingDriveCommand(
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightY(),
        () -> -driverController.getRightX(),
        driverController::getLeftTriggerAxis,
        driverController::getRightTriggerAxis);
    swerve.setDefaultCommand(headingSteeringCommand);
    swerve.setupPathPlanner();

    SmartDashboard.putNumber("speed", 0.25);
    Shuffleboard.getTab("Config").add("Zero swerve offsets",
        swerve.runOnce(() -> swerve.setSwerveOffsets()).ignoringDisable(true));
    Shuffleboard.getTab("Config").add("Set offsets to 0",
        swerve.runOnce(() -> swerve.zeroSwerveOffsets()).ignoringDisable(true));
    Shuffleboard.getTab("Config").add("Zero gyro", swerve.runOnce(() -> swerve.zeroGyro()).ignoringDisable(true));

    SmartDashboard.putNumber("Algae Intake Motor Speed", 0.25);
    SmartDashboard.putNumber("Algae Pivot Speed", 0.25);
    SmartDashboard.putNumber("Coral Outtake Speed", 0.25);
    SmartDashboard.putNumber("Elevator Motor Speed", 0.25);
    SmartDashboard.putNumber("Elevator Sled Speed", 0.25);
    SmartDashboard.putNumber("Climber Speed", 0.25);
    SmartDashboard.putNumber("Intake Motor Speed", 0.25);
    
    Shuffleboard.getTab("testing").add("Algae Motor Speed", 0.25);
    Shuffleboard.getTab("testing").add("Algae Motor", algae.dynamicAlgaePickup(
      () -> SmartDashboard.getNumber("Algae Intake Motor Speed", 0.25)));
    Shuffleboard.getTab("testing").add("Algae Pivot", algae.dynamicAlgaeSpeedPivot(
      () -> SmartDashboard.getNumber("Algae Pivot Speed", 0.25)));
    Shuffleboard.getTab("testing").add("Coral Outtake", coral.dynamicDriveCoral(
      () -> SmartDashboard.getNumber("Coral Outtake Speed", -0.25)));
    Shuffleboard.getTab("testing").add("Elevator Motor", elevator.dynamicElevatorSetSpeed(
      () -> SmartDashboard.getNumber("Elevator Motor Speed", 0.25)));
    // Shuffleboard.getTab("testing").add("Shuttle Motor", elevator.dynamicShuttleSetSpeed(
    //   () -> SmartDashboard.getNumber("Elevator Sled Speed", 0.25)));
    Shuffleboard.getTab("testing").add("Climber Motor", climber.dynamicDriveClimb(
      () -> SmartDashboard.getNumber("Climber Speed", 0.25)));
    Shuffleboard.getTab("testing").add("Intake Motor", intake.dynamicDriveIntake(
      () -> SmartDashboard.getNumber("Intake Motor Speed", 0.25)));


    // Configure the trigger bindings
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);


    Shuffleboard.getTab("Debug").addString("Selected Node", objectiveTracker::getObjectiveString);
  }

  
  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    /*
     * Operator Bindings:
     * left trigger: hold to climb
     * right trigger: coral output
     * "menu" button: elevator to ground
     * dpad: move node selector
     * Y: algae output (arm up & run motor reverse)
     * X: algae arm up
     * B: run coral intake :)
     * A: algae intake (arm down & run motor)
     */
    
    //Operator Controls --------------------------------------------------------
    //Coral Inputs
    operatorController.b().whileTrue(intake.startStopIntake(
      () -> SmartDashboard.getNumber("Intake Motor Speed", 0.25)));

    operatorController.rightTrigger().whileTrue(coral.startStopCoral(
      () -> SmartDashboard.getNumber("Coral Outtake Speed", 0.25)));

    //Algae Arm Inputs:
    operatorController.x().whileTrue(algae.startStopAlgaePivot(
      () -> SmartDashboard.getNumber("Algae Active Angle", 0)));

    operatorController.a().whileTrue(algae.dualAlgaeIntake(
      () -> SmartDashboard.getNumber("Algae Up Angle", 0),
       () -> SmartDashboard.getNumber("Algae Speed", 0.25)));

    operatorController.y().whileTrue(algae.dualAlgaeIntake(
      () -> SmartDashboard.getNumber("Algae Down Angle", 0),
       () -> SmartDashboard.getNumber("Algae Speed", 0.25) * -1));

    //Elevator Inputs:
    operatorController.back().onTrue(elevator.homeElevator());

    //Climber Inputs:
    operatorController.leftTrigger().whileTrue(climber.startStopClimb(
        () -> SmartDashboard.getNumber("Climber Active Angle", 0)))
      .onFalse(climber.startStopClimb(
        () -> SmartDashboard.getNumber("Climber Default Angle", 0)));

    // enhanced controls through objective tracker
    operatorController.povUp().onTrue(objectiveTracker.moveIndex(MoveDirection.UP));
    operatorController.povDown().onTrue(objectiveTracker.moveIndex(MoveDirection.DOWN));
    operatorController.povRight().onTrue(objectiveTracker.moveIndex(MoveDirection.RIGHT));
    operatorController.povLeft().onTrue(objectiveTracker.moveIndex(MoveDirection.LEFT));

    //Driver Controls ----------------------------------------------------------
    // Drive to reef:
    driverController.x().onTrue(swerve.driveToReef(objectiveTracker::isRightSide));
    // Drive Intake:
    driverController.b().whileTrue(intake.dynamicDriveIntake(
        () -> SmartDashboard.getNumber("Intake Speed", 0.25)));
    // Drive to nearest coral station:
    driverController.y().onTrue(swerve.driveToNearestCoralStation());
    // Drive to algae processor:
    driverController.leftTrigger().onTrue(swerve.driveToAlgaeCollector());

    // Slow driving
    driverController.a().onTrue(swerve.enableSlowDriving()).onFalse(swerve.disableSlowDriving());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    /**Selects all autonomous paths; selectable from smart dashboard*/
  }

  public void setInitialRobotPose(Pose2d pose) {
    swerve.setPose(pose);
  }
}
