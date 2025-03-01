// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.Set;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.AllDeadbands;
import frc.robot.lib.VisionData;
import frc.robot.lib.util.AllianceFlipUtil;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.parser.SwerveModuleConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

    private final StructPublisher<Pose2d> rightPose;
    private final StructPublisher<Pose2d> leftPose;

    private final SwerveDrive swerveDrive;
    public double maximumSpeed = SwerveConstants.SWERVE_MAXIMUM_VELOCITY;
    public double maximumAngularSpeed = SwerveConstants.SWERVE_MAXIMUM_ANGULAR_VELOCITY;
    private Rotation2d currentTargetAngle = new Rotation2d();
    private boolean hadbadreading;

    /** Creates a new Swerve. */
    public Swerve(File config_dir) {

        rightPose = NetworkTableInstance.getDefault()
            .getStructTopic("ReefTargetPoses/right", Pose2d.struct).publish();
        leftPose = NetworkTableInstance.getDefault()
            .getStructTopic("ReefTargetPoses/left", Pose2d.struct).publish();

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.POSE;
        try {
            swerveDrive = new SwerveParser(config_dir).createSwerveDrive(maximumSpeed);
            // Alternative method if you don't want to supply the conversion factor via JSON
            // files.
            // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
            // angleConversionFactor, driveConversionFactor);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
        swerveDrive.setHeadingCorrection(false); // Heading correction should only be used while controlling the robot
                                                 // via angle.
        // swerveDrive.chassisVelocityCorrection = false;

        for (int i = 0; i < swerveDrive.getModules().length; i++) {
            // Lower the kS to reduce wobble?
            // swerveDrive.getModules()[i].setFeedforward(new
            // SimpleMotorFeedforward(SwerveConstants.MODULE_CONSTANTS[i].drive_kS * 0.1,
            // SwerveConstants.MODULE_CONSTANTS[i].drive_kV,
            // SwerveConstants.MODULE_CONSTANTS[i].drive_kA));
        }
        // setupPathPlanner();
    }
    public void setupPathPlanner(){
        RobotConfig config = null;
    try{
      config = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Configure AutoBuilder last
    AutoBuilder.configure(
            swerveDrive::getPose, // Robot pose supplier
            swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
            swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> swerveDrive.drive(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> {
              // Boolean supplier that controls when the path will be mirrored for the red alliance
              // This will flip the path being followed to the red side of the field.
              // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
            },
            this // Reference to this subsystem to set requirements
    );
  }


    /** Gets the current alliance, defaulting to blue */
    public Alliance getAllianceDefaultBlue() {
        Alliance currentAlliance;
        if (DriverStation.getAlliance().isPresent()) {
            currentAlliance = DriverStation.getAlliance().get();
        } else {
          currentAlliance = Alliance.Blue;
          //System.out.println("No alliance, setting to blue");
        }
        return currentAlliance;
    }

    /** Flips a Pose2d by 180 degrees if necessary */
    public Pose2d alliancePoseFlipper(Pose2d input) {
        if (getAllianceDefaultBlue() == Alliance.Blue) {
            return input;
        } else {
            return new Pose2d(FieldConstants.fieldLength - input.getX(), FieldConstants.fieldWidth - input.getY(),
                    input.getRotation().minus(new Rotation2d(Math.PI)));
        }
    }

    /** Takes a chassis speeds object and flips it 180 degrees if needed */
    public ChassisSpeeds allianceTargetSpeedsFlipper(ChassisSpeeds input) {
        if (getAllianceDefaultBlue() == Alliance.Blue) {
            return input;
        } else {
            return new ChassisSpeeds(-input.vxMetersPerSecond, -input.vyMetersPerSecond, input.omegaRadiansPerSecond);
        }
    }

    /** Takes a rotation2d and flips it 180 degrees */
    public Rotation2d allianceRotationFlipper(Rotation2d input) {
        return getAllianceDefaultBlue() == Alliance.Blue ? input : input.minus(new Rotation2d(Math.PI));
    }

    /**
     * Drive robot with alliance relative speeds.
     * Converts them to field relative speeds first then drives the robot.
     * If using heading drive, it does not use the target angle at all.
     */
    public void driveAllianceRelative(double x, double y, double rotationRate, boolean headingDrive) {
        if (headingDrive) {
            swerveDrive.driveFieldOriented(
                    swerveDrive.swerveController.getTargetSpeeds(getAllianceDefaultBlue() == Alliance.Blue ? x : -x,
                            getAllianceDefaultBlue() == Alliance.Blue ? y : -y,
                            currentTargetAngle.getRadians(), swerveDrive.getYaw().getRadians(), maximumSpeed));
        } else {
            swerveDrive.drive(
                    new Translation2d(getAllianceDefaultBlue() == Alliance.Blue ? x : -x,
                            getAllianceDefaultBlue() == Alliance.Blue ? y : -y),
                    rotationRate, true, false);
        }
    }

    /**
     * Resets the gyro angle to zero and resets odometry to the same position, but
     * facing toward 0. (Alliance Relative)
     */
    public void zeroGyro() {
        setGyroAngle(allianceRotationFlipper(new Rotation2d()));
        currentTargetAngle = allianceRotationFlipper(new Rotation2d());
        swerveDrive.resetOdometry(
                new Pose2d(swerveDrive.getPose().getTranslation(), allianceRotationFlipper(new Rotation2d())));
    }

    /**
     * Sets the current robot pose.
     * This should not typically be called -- it is exposed to allow the robot
     * to be set to a valid position on the field when starting a simulation.
     * 
     * @param pose The current robot pose.
     */
    public void setPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    public Pose2d getPose() {
        return swerveDrive.getPose();
    }

    /**
     * Sets the current angle of the gyro (Field Relative). If the robot reaches the
     * same angle, the
     * gyro will report this angle.
     * 
     * @param currentAngle The angle that the gyro should read in its current state.
     */
    public void setGyroAngle(Rotation2d currentAngle) {
        swerveDrive.setGyro(new Rotation3d(0, 0, currentAngle.getRadians()));
    }

    public Rotation2d getGyroAngle(){
        return swerveDrive.getYaw();
    }

    public double getAngularVelocityRad_Sec() {
        return swerveDrive.getRobotVelocity().omegaRadiansPerSecond;
    }

    public Rotation2d getCurrentTargetAngle() {
        return currentTargetAngle;
    }

    public void setTargetAngle(Rotation2d newTargetAngle) {
        currentTargetAngle = newTargetAngle;
    }

    public void setTargetAllianceRelAngle(Rotation2d allianceRelAngle) {
        currentTargetAngle = allianceRotationFlipper(allianceRelAngle);
    }

    /**
     * <h2>More features</h2>
     * Drives alliance relative
     * 
     * @param translationX      Translation input in the X direction.
     * @param translationY      Translation input in the Y direction.
     * @param rotationX         Rotation input in the X direction.
     * @param rotationY         Rotation input in the Y direction.
     * @param leftRotationRate  Left rotation input that overrides heading angle.
     * @param rightRotationRate Right rotation input that overrides heading angle.
     * @return A better combined drive command.
     */
    public Command enhancedHeadingDriveCommand(DoubleSupplier translationX, DoubleSupplier translationY,
            DoubleSupplier rotationX, DoubleSupplier rotationY,
            DoubleSupplier leftRotationRate, DoubleSupplier rightRotationRate) {
        return run(() -> {
            double[] deadbandRotationInputs = AllDeadbands
                    .applyCircularDeadband(new double[] { rotationX.getAsDouble(), rotationY.getAsDouble() }, 0.95);
            double leftRotationInput = MathUtil.applyDeadband(leftRotationRate.getAsDouble(), 0.05);
            double rightRotationInput = MathUtil.applyDeadband(rightRotationRate.getAsDouble(), 0.05);
            double rawXInput = translationX.getAsDouble();
            double rawYInput = translationY.getAsDouble();
            double[] scaledDeadbandTranslationInputs = AllDeadbands
                    .applyScaledSquaredCircularDeadband(new double[] { rawXInput, rawYInput }, 0.1);
            double xInput = scaledDeadbandTranslationInputs[0];
            double yInput = scaledDeadbandTranslationInputs[1];
            // determining if target angle is commanded
            if (deadbandRotationInputs[0] != 0 || deadbandRotationInputs[1] != 0) {
                setTargetAllianceRelAngle(
                        Rotation2d.fromRadians(Math.atan2(deadbandRotationInputs[1], deadbandRotationInputs[0])));
            }
            if (leftRotationInput != 0 && rightRotationInput == 0) {
                swerveDrive.setHeadingCorrection(false);
                double leftRotationOutput = Math.pow(leftRotationInput, 3) * maximumAngularSpeed;
                driveAllianceRelative(xInput * maximumSpeed, yInput * maximumSpeed, leftRotationOutput, false);
            }
            // If right trigger pressed, rotate left at a rate proportional to the right
            // trigger input
            else if (rightRotationInput != 0 && leftRotationInput == 0) {
                swerveDrive.setHeadingCorrection(false);
                double rightRotationOutput = -Math.pow(rightRotationInput, 3) * maximumAngularSpeed;
                driveAllianceRelative(xInput * maximumSpeed, yInput * maximumSpeed, rightRotationOutput, false);
                currentTargetAngle = null;
            }
            // If no triggers are pressed or both are pressed, use the right stick for
            // heading angle steering
            else {
                // If there is no current target angle (last action was spin), then don't
                // command the angle

                swerveDrive.setHeadingCorrection(currentTargetAngle != null);
                if (currentTargetAngle != null) {
                    // Make the robot move
                    driveAllianceRelative(xInput, yInput, currentTargetAngle.getRadians(), true);
                } else {
                    driveAllianceRelative(xInput * maximumSpeed, yInput * maximumSpeed, 0, false);
                }
            }
        });
    }

    // * Adds vision measurement from vision object to swerve
    public void addVisionData(VisionData visionData) {
        Pose2d swervePose = swerveDrive.getPose();
        double previousx = swervePose.getX();
        double previousy = swervePose.getY();
        Rotation2d previousTheta = swerveDrive.getYaw();
        if (Double.isNaN(previousy) || Double.isNaN(previousx)) {
            hadbadreading = true;
            previousx = 0;
            previousy = 0;
            System.out.println("Swerve Pose is NaN");
        }

        if (Double.isNaN(visionData.visionPose().getX()) || Double.isNaN(visionData.visionPose().getY())) {
            System.out.println("Recived a bad vision pose");
            return;
        }

        if (hadbadreading) {
            swerveDrive.resetOdometry(new Pose2d(0.0, 0.0, swerveDrive.getYaw()));
        }

        // Add Vision Measurement if it passes the checks, but without taking into
        // account vision yaw.
        swerveDrive.addVisionMeasurement(
                new Pose2d(visionData.visionPose().getTranslation(), swerveDrive.getYaw()),
                visionData.time(),
                visionData.visionReliability());
        Pose2d newPose = swerveDrive.getPose();
        if (Double.isNaN(newPose.getX()) || Double.isNaN(newPose.getY())) {
            // hadbadreading = true;
            Pose2d pose = new Pose2d(previousx, previousy, previousTheta);
            swerveDrive.resetOdometry(pose);
            System.out.println("Vision pose was invalid and not caught");
        }
    }

    public Command driveToReef(BooleanSupplier isRightSideSupplier) {
        return driveToPoseFlipped(() -> reefLocation(isRightSideSupplier)).finallyDo(() -> setTargetAngle(reefLocation(isRightSideSupplier).getRotation()));
    }

    /**
   * Returns the pose that the robot should pathfind to for a particular reef side on the left or right. Reef side can be returned by findNearestReefSide
   * @param reefSide goes from 1 to 6, starting from the side closest to the alliance station and going counterclockwise
   * @param isRightSide is true if we're on the right side of the specified reef side
   * @return a Pose2d representing the location and orientation of the robot if facing the reef on the specified 
   */
  public Pose2d reefLocation(BooleanSupplier isRightSideSupplier) {
    int reefSide = FieldConstants.findNearestReefSide(swerveDrive.getPose());

    int poseCode = (1 <= reefSide && reefSide <= 6) ? (reefSide * 2 + (isRightSideSupplier.getAsBoolean() ? 1 : 0)) : -1;
    Rotation2d angle = Rotation2d.fromDegrees(switch(reefSide) {
      case 1 -> 0;
      case 2 -> 60;
      case 3 -> 120;
      case 4 -> 180;
      case 5 -> 240;
      case 6 -> 300;
      default -> 0;
    });

    double[] pos = switch(poseCode) {
      case 2  -> new double[] { 158.00, 164.94 };
      case 3  -> new double[] { 158.00, 152.06 };
      case 4  -> new double[] { 168.80, 133.36 };
      case 5  -> new double[] { 179.95, 126.92 };
      case 6  -> new double[] { 201.55, 126.92 };
      case 7  -> new double[] { 212.70, 133.36 };
      case 8  -> new double[] { 223.50, 152.06 };
      case 9  -> new double[] { 223.50, 164.94 };
      case 10 -> new double[] { 212.70, 183.64 };
      case 11 -> new double[] { 201.55, 190.08 };
      case 12 -> new double[] { 179.95, 190.08 };
      case 13 -> new double[] { 168.80, 183.64 };
      default -> new double[] { 0, 0 };
    };

    Pose2d pose = new Pose2d(Units.inchesToMeters(pos[0]), Units.inchesToMeters(pos[1]), angle);

    //back up pose by 16" so it's not overlapping the reef
    pose = pose.transformBy(new Transform2d(new Translation2d(-FieldConstants.reefLocationBackupDistance, 0), new Rotation2d()));

    return pose;
  }


    /**
     * Command to characterize the robot drive motors using SysId
     *
     * @return SysId Drive Command
     */
    public Command sysIdDriveMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setDriveSysIdRoutine(
                        new Config(),
                        this, swerveDrive, 12, false),
                3.0, 5.0, 3.0); // TODO: Tweak (increase quasitimeout if possible) for running sysid
                                // characterization
    }

    /**
     * Command to characterize the robot angle motors using SysId
     *
     * @return SysId Angle Command
     */
    public Command sysIdAngleMotorCommand() {
        return SwerveDriveTest.generateSysIdCommand(
                SwerveDriveTest.setAngleSysIdRoutine(
                        new Config(),
                        this, swerveDrive),
                3.0, 5.0, 3.0); // TODO: Tweak (increase quasitimeout if possible) if needed for running sysid
                                // characterization
    }

    public boolean isvisionOk() {
        return !hadbadreading;
    }



    /**
     * Set the hardware zero point of the angle motor's absolute encoder to the
     * current position.
     * The 45 + 90n degree angle offsets are in software, on top of this.
     */
    public void zeroSwerveOffsets() {
        for (SwerveModule module : swerveDrive.getModules()) {
            // double position = module.getRawAbsolutePosition();
            // double encoderOffset = ((SparkMax)
            // module.configuration.angleMotor.getMotor()).configAccessor.absoluteEncoder.getZeroOffset();
            // module.configuration.absoluteEncoder.setAbsoluteEncoderOffset(MathUtil.inputModulus((encoderOffset
            // + position) / 360, 0, 1));
            module.configuration.absoluteEncoder.setAbsoluteEncoderOffset(0);
            System.out.println("absolute position:" + module.configuration.absoluteEncoder.getAbsolutePosition());
            System.out.println("raw ap:" + module.getRawAbsolutePosition());
        }
    }

    public void setSwerveOffsets() {
        Rotation2d[] currentOffsets = new Rotation2d[4];
        Rotation2d[] newOffsets = new Rotation2d[4];
        Rotation2d[] measuredPositions = new Rotation2d[4];
        double[] angleOffsets = new double[4];
        SparkAbsoluteEncoder[] encoders = new SparkAbsoluteEncoder[4];
        SwerveModuleConfiguration[] moduleConfigs = new SwerveModuleConfiguration[4];
        SparkMax[] angleMotors = new SparkMax[4];
        for (int i = 0; i < 4; i++) {
            moduleConfigs[i] = swerveDrive.getModules()[i].configuration;
            angleMotors[i] = (SparkMax) swerveDrive.getModules()[i].getAngleMotor().getMotor();
            angleOffsets[i] = angleMotors[i].configAccessor.absoluteEncoder.getZeroOffset() * 360;
            currentOffsets[i] = Rotation2d.fromDegrees(angleOffsets[i]);

            encoders[i] = (SparkAbsoluteEncoder) swerveDrive.getModules()[i].getAbsoluteEncoder().getAbsoluteEncoder();
            measuredPositions[i] = Rotation2d.fromDegrees(encoders[i].getPosition());
            newOffsets[i] = new Rotation2d().plus(currentOffsets[i]).plus(measuredPositions[i])
                    .plus(Rotation2d.fromDegrees(getAngleForModule(i)));
            moduleConfigs[i].absoluteEncoder
                    .setAbsoluteEncoderOffset(MathUtil.inputModulus(newOffsets[i].getDegrees() / 360, 0, 1));
            System.out.println("Module " + i + " offset: " + newOffsets[i].getDegrees());
            System.out.println(currentOffsets[i].getDegrees() + " + " + measuredPositions[i].getDegrees() + " = "
                    + newOffsets[i].getDegrees());
        }
    }

    private double getAngleForModule(int moduleNumber) {
        return switch (moduleNumber) {
            case 0 -> 225;
            case 1 -> 315;
            case 2 -> 135;
            case 3 -> 45;
            default -> throw new IllegalArgumentException("Invalid module number");
        };
    }

    public Command driveForward(double percentage) {
        return run(() -> {
            swerveDrive.drive(new Translation2d(percentage * maximumSpeed, 0), 0, false, false);
        });
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        swerveDrive.updateOdometry();
        // Output Reef Poses - Remove this later!!! (Or else...)
        Pose2d rightTarget = reefLocation(() -> true);
        rightPose.set(rightTarget);

        Pose2d leftTarget = reefLocation(() -> false);
        leftPose.set(leftTarget);
    }

    /** Drive to a pose, flipped if on red alliance */
    public Command driveToPoseFlipped(Supplier<Pose2d> poseSupplier) {
        PathConstraints constraints = PathConstraints.unlimitedConstraints(12);
        return Commands.defer(() -> AutoBuilder.pathfindToPoseFlipped(poseSupplier.get(), constraints), Set.of(this));
    }

    // *******************
    // Logging methods
    // *******************
    @Logged(name = "Rotation Degrees")
    public double getRotationDegrees() {
        return swerveDrive.getYaw().getDegrees();
    }

    @Logged(name="FR Drive Motor")
    public SparkMax getFrontRightDriveMotor() {
        return getDriveMotor(SwerveConstants.FRONT_RIGHT_MODULE_INDEX);
    }

    @Logged(name="FR Angle Motor")
    public SparkMax getFrontRightAngleMotor() {
        return getAngleMotor(SwerveConstants.FRONT_RIGHT_MODULE_INDEX);
    }

    @Logged(name="FL Drive Motor")
    public SparkMax getFrontLeftDriveMotor() {
        return getDriveMotor(SwerveConstants.FRONT_LEFT_MODULE_INDEX);
    }

    @Logged(name="FL Angle Motor")
    public SparkMax getFrontLeftAngleMotor() {
        return getAngleMotor(SwerveConstants.FRONT_LEFT_MODULE_INDEX);
    }

    @Logged(name="BR Drive Motor")
    public SparkMax getBackRightDriveMotor() {
        return getDriveMotor(SwerveConstants.BACK_RIGHT_MODULE_INDEX);
    }

    @Logged(name="BR Angle Motor")
    public SparkMax getBackRightAngleMotor() {
        return getAngleMotor(SwerveConstants.BACK_RIGHT_MODULE_INDEX);
    }

    @Logged(name="BL Drive Motor")
    public SparkMax getBackLeftDriveMotor() {
        return getDriveMotor(SwerveConstants.BACK_LEFT_MODULE_INDEX);
    }

    @Logged(name="BL Angle Motor")
    public SparkMax getBackLeftAngleMotor() {
        return getAngleMotor(SwerveConstants.BACK_LEFT_MODULE_INDEX);
    }

    private SparkMax getDriveMotor(int swerveModuleIndex) {
        return (SparkMax) swerveDrive.getModules()[swerveModuleIndex].getDriveMotor().getMotor();
    }

    private SparkMax getAngleMotor(int swerveModuleIndex) {
        return (SparkMax) swerveDrive.getModules()[swerveModuleIndex].getAngleMotor().getMotor();
    }
}
