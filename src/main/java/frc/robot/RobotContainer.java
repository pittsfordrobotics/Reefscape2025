// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AlgaeConstants;
import frc.robot.Constants.CoralConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Elevator.ElevatorLevels;
import frc.robot.subsystems.Vision.Vision;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Coral;

import frc.robot.subsystems.objectiveTracker.ObjectiveSelecterIONetworkTables;
import frc.robot.subsystems.objectiveTracker.ObjectiveTracker;
import frc.robot.subsystems.objectiveTracker.ObjectiveSelectorIO.MoveDirection;

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
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AlgaeCommands;
import frc.robot.commands.IntakeCoral;

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

  @Logged(name = "Algae Subsystem")
  private final Algae algae;
  private final SendableChooser<Command> autoChooser;
  private final ObjectiveTracker objectiveTracker;

  //@Logged(name = "Climber Subsystem")
  //private final Climber climber;

  @Logged(name = "Elevator Subsystem")
  private final Elevator elevator;

  @Logged(name = "Coral Output Subsystem")
  private final Coral coral;
  
  @Logged(name = "Swerve Subsystem")
  private final Swerve swerve;

  @SuppressWarnings("unused") //it is used, vs code is just crashing out
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
    algae = new Algae();
    //climber = new Climber();
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

    algae.setDefaultCommand(algae.dynamicAlgaeSetPivot(() -> AlgaeConstants.PIVOT_STORE_DEGREES));

    Shuffleboard.getTab("Config").add("Zero swerve offsets",
        swerve.runOnce(() -> swerve.setSwerveOffsets()).ignoringDisable(true));
    Shuffleboard.getTab("Config").add("Set offsets to 0",
        swerve.runOnce(() -> swerve.zeroSwerveOffsets()).ignoringDisable(true));
    Shuffleboard.getTab("Config").add("Zero gyro", swerve.runOnce(() -> swerve.zeroGyro()).ignoringDisable(true));
    
    NamedCommands.registerCommand("stopSwerve", Commands.runOnce(swerve::stopSwerve));
    NamedCommands.registerCommand("holdHeading", swerve.setTargetAngleCommand(swerve::getGyroAngle));
    NamedCommands.registerCommand("dropCoralTrough", coral.placeCoral());
    NamedCommands.registerCommand("coralDrop", coral.placeCoral());
    NamedCommands.registerCommand("collectCoral", new IntakeCoral(coral, elevator));
    NamedCommands.registerCommand("ElevatorL4", elevator.setElevatorLevel(ElevatorLevels.L4).andThen(Commands.waitUntil(() -> elevator.isAtLevel(() -> ElevatorLevels.L4))));
    NamedCommands.registerCommand("ElevatorL3", elevator.setElevatorLevel(ElevatorLevels.L3).andThen(Commands.waitUntil(() -> elevator.isAtLevel(() -> ElevatorLevels.L3))));
    NamedCommands.registerCommand("ElevatorL2", elevator.setElevatorLevel(ElevatorLevels.L2).andThen(Commands.waitUntil(() -> elevator.isAtLevel(() -> ElevatorLevels.L2))));
    NamedCommands.registerCommand("ElevatorIntake", elevator.setElevatorLevel(ElevatorLevels.INTAKE).andThen(Commands.waitUntil(() -> elevator.isAtLevel(() -> ElevatorLevels.INTAKE))));
    
    Shuffleboard.getTab("Config").add("SysID Drive Motors", swerve.sysIdDriveMotorCommand());
    Shuffleboard.getTab("Config").add("SysID Angle Motors", swerve.sysIdAngleMotorCommand());


    initTestingDashboards();

    // Configure the trigger bindings
    configureBindings();
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
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
    operatorController.b().whileTrue(new IntakeCoral(coral, elevator)).onFalse(coral.stopCoral());

    operatorController.rightTrigger().whileTrue(coral.dynamicDriveCoral(() -> CoralConstants.CORAL_SPEED));

    //Algae Arm Inputs:
    algae.setDefaultCommand(algae.dynamicAlgaeSetPivot(() -> AlgaeConstants.PIVOT_STORE_DEGREES));
    operatorController.x().whileTrue(AlgaeCommands.algaeIntake(algae, elevator, objectiveTracker));
    operatorController.y().whileTrue(AlgaeCommands.algaeOuttake(algae, elevator, objectiveTracker)).onFalse(algae.idleAlgae());

    // operatorController.a().whileTrue(algae.dualAlgaeIntake(
    //   () -> SmartDashboard.getNumber("Algae Up Angle", 0),
    //   () -> SmartDashboard.getNumber("Algae Speed", 0.25)));

    // operatorController.y().whileTrue(algae.dualAlgaeIntake(
    //   () -> SmartDashboard.getNumber("Algae Down Angle", 0),
    //   () -> SmartDashboard.getNumber("Algae Speed", 0.25) * -1));

    //Elevator Inputs:
    // operatorController.back().onTrue(elevator.homeElevator());
    operatorController.back().onTrue(elevator.dynamicElevatorLevel(() -> ElevatorLevels.ZERO));
    operatorController.a().onTrue(elevator.dynamicElevatorLevel(() -> objectiveTracker.getElevatorLevel()));
    // operatorController.povUp().onTrue(elevator.dynamicElevatorLevel(() -> ElevatorLevels.L1));
    // operatorController.povDown().onTrue(elevator.dynamicElevatorLevel(() -> ElevatorLevels.L2));
    // operatorController.povLeft().onTrue(elevator.dynamicElevatorLevel(() -> ElevatorLevels.L3));
    // operatorController.povRight().onTrue(elevator.dynamicElevatorLevel(() -> ElevatorLevels.L4));
    operatorController.rightBumper().onTrue(Commands.runOnce(
      (() -> elevator.increaseEncoderOffset((int)SmartDashboard.getNumber("Elevator Encoder Offset", 2)))));
    operatorController.leftBumper().onTrue(Commands.runOnce((() -> elevator.zeroElevatorOffset())));

    //operatorController.leftTrigger().whileTrue(climber.dynamicDriveClimb(() -> -0.33));// TODO: fix

    //Climber Inputs:
    // operatorController.leftTrigger().whileTrue(climber.dynamicDriveClimb(
    //     () -> SmartDashboard.getNumber("Climber Active Angle", 0))) // not defined in dashboard x2????
    //   .onFalse(climber.stopClimb());

    // enhanced controls through objective tracker
    operatorController.povUp().onTrue(objectiveTracker.moveIndex(MoveDirection.UP));
    operatorController.povDown().onTrue(objectiveTracker.moveIndex(MoveDirection.DOWN));
    operatorController.povRight().onTrue(objectiveTracker.moveIndex(MoveDirection.RIGHT));
    operatorController.povLeft().onTrue(objectiveTracker.moveIndex(MoveDirection.LEFT));

    //Driver Controls ----------------------------------------------------------
    // Drive to reef:
    driverController.x().whileTrue(swerve.driveToReef(objectiveTracker::isRightSide));
    // Drive Intake:
    driverController.b().whileTrue(swerve.driveToAlgaeCollector());
    // Drive to nearest coral station:
    driverController.y().whileTrue(swerve.driveToNearestCoralStation());

    // Zero gyro:
    driverController.povUp().onTrue(swerve.runOnce(() -> swerve.zeroGyro()).ignoringDisable(true));

    // Slow driving
    driverController.a().onTrue(Commands.runOnce((() -> swerve.enableSlowDriving()))).onFalse(Commands.runOnce((() -> swerve.disableSlowDriving())));
    elevator.forceSlowDrive().onTrue(Commands.runOnce(() -> swerve.enableSlowDriving())).onFalse(Commands.runOnce((() -> swerve.disableSlowDriving())));
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

  private void initTestingDashboards(){
    Shuffleboard.getTab("testing").add("Algae Motor Speed", 0.25);
    // Shuffleboard.getTab("testing").add("Algae Motor", algae.dynamicAlgaePickup(
    //   () -> SmartDashboard.getNumber("Algae Intake Motor Speed", 0.25)));
    Shuffleboard.getTab("testing").add("Algae Pivot", algae.dynamicAlgaeSpeedPivot(
      () -> SmartDashboard.getNumber("Algae Pivot Speed", 0.25)));
    Shuffleboard.getTab("testing").add("Coral Outtake", coral.dynamicDriveCoral(
      () -> SmartDashboard.getNumber("Coral Outtake Speed", -0.25)));
    Shuffleboard.getTab("testing").add("Elevator Motor", elevator.dynamicElevatorSetSpeed(
      () -> SmartDashboard.getNumber("Elevator Motor Speed", 0.25)));
    //Shuffleboard.getTab("testing").add("Climber Motor", climber.dynamicDriveClimb(
    //  () -> SmartDashboard.getNumber("Climber Speed", 0.25)));
    // SmartDashboard.putNumber("Climber Speed", 0.25);

    Shuffleboard.getTab("testing").add("Elevator Encoder Offset", 2);
    Shuffleboard.getTab("Debug").addString("Selected Node", objectiveTracker::getObjectiveString);

  }
}
