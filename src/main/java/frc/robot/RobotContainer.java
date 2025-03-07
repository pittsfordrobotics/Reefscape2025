// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelecterIONetworkTables;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelectorIO.MoveDirection;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;

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

    swerve = new Swerve(new File(Filesystem.getDeployDirectory(), "swerve"));
    coral = new Coral();

    vision = new Vision(
      VisionConstants.LIMELIGHT_LEFT,
      VisionConstants.LIMELIGHT_RIGHT,
      VisionConstants.LIMELIGHT_BACK,
      swerve::getGyroAngle,
      swerve::getAngularVelocityRad_Sec,
      swerve::addVisionData);

    SmartDashboard.putNumber("Algae Speed", 0.25);

    SmartDashboard.putNumber("Algae Pivot Speed", 0.25);
    SmartDashboard.putNumber("Algae Active Angle", 0);
    SmartDashboard.putNumber("Algae Default Angle", 0);
    
    SmartDashboard.putNumber("Climb Speed", 0.25);
    SmartDashboard.putNumber("Climb Default Angle", 0);
    SmartDashboard.putNumber("Climb Active Angle", 0);
    

    ObjectiveSelecterIONetworkTables objectiveSelecterIOImpl = new ObjectiveSelecterIONetworkTables();
    objectiveTracker = new ObjectiveTracker(objectiveSelecterIOImpl);
    objectiveTracker.setDefaultCommand(objectiveTracker.updateReefSide(swerve::getPose));
    
    Command enhancedHeadingSteeringCommand = swerve.enhancedHeadingDriveCommand(
        () -> -driverController.getLeftY(),
        () -> -driverController.getLeftX(),
        () -> -driverController.getRightY(),
        () -> -driverController.getRightX(),
        driverController::getLeftTriggerAxis,
        driverController::getRightTriggerAxis);
    swerve.setDefaultCommand(enhancedHeadingSteeringCommand);
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
    
    
    NamedCommands.registerCommand("dropCoralTrough", coral.placeCoral());
    NamedCommands.registerCommand("coralDrop", coral.placeCoral());
    NamedCommands.registerCommand("collectCoral", intake.intakeCoralWithSensor());



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
    /**
     * Isaac's requested bindings:
     * left trigger: hold to climb
     * right trigger: coral output
     * "menu" button (third tiny button in the middle): elevator to ground
     * dpad: move node selector
     * Y: algae output (arm up & run motor reverse)
     * X: algae arm up
     * B: run coral intake
     * A: algae intake (arm down & run motor)
     */
    
    //operator controls
    operatorController.b().whileTrue(intake.dynamicDriveIntake(
      () -> SmartDashboard.getNumber("Intake Motor Speed", 0.25)))
      .onFalse(intake.stopIntake());
    operatorController.rightTrigger().whileTrue(coral.dynamicDriveCoral(
      () -> SmartDashboard.getNumber("Coral Outtake Speed", 0.25)))
      .onFalse(coral.stopCoral());

      // operator controls moved to the objective tracker
    // operatorController.povUp().onTrue(elevator.setElevatorLevel(ElevatorLevels.INTAKE));
    // operatorController.povLeft().onTrue(elevator.setElevatorLevel(ElevatorLevels.L2));
    // operatorController.povDown().onTrue(elevator.setElevatorLevel(ElevatorLevels.L3));
    // operatorController.povRight().onTrue(elevator.setElevatorLevel(ElevatorLevels.L4));
    // Drive to reef:
    driverController.x().onTrue(swerve.driveToReef(objectiveTracker::isRightSide));
    // Drive Intake:
    driverController.b().whileTrue(intake.dynamicDriveIntake(
        () -> SmartDashboard.getNumber("Intake Speed", 0.25)));

    // Pivot Algae arm:
    // Pos 1
    driverController.rightTrigger().onTrue(algae.dynamicAlgaeSpeedPivot(
        () -> SmartDashboard.getNumber("Algae Angle 1", 0)));
    // Pos 2
    driverController.rightBumper().onTrue(algae.dynamicAlgaeSpeedPivot(
        () -> SmartDashboard.getNumber("Algae Angle 2", 0)));

    // Drive Algae pickup:
    driverController.a().whileTrue(algae.dynamicAlgaePickup(() -> SmartDashboard.getNumber("Algae Speed", 0.25)));

    // enhanced controls through objective tracker
    operatorController.povUp().onTrue(objectiveTracker.moveIndex(MoveDirection.UP));
    operatorController.povDown().onTrue(objectiveTracker.moveIndex(MoveDirection.DOWN));
    operatorController.povRight().onTrue(objectiveTracker.moveIndex(MoveDirection.RIGHT));
    operatorController.povLeft().onTrue(objectiveTracker.moveIndex(MoveDirection.LEFT));
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
