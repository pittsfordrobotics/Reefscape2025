// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.File;
import java.util.function.DoubleSupplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;
import com.revrobotics.AbsoluteEncoder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.lib.AllDeadbands;
import frc.robot.lib.VisionData;
import frc.robot.lib.util.AllianceFlipUtil;
import swervelib.SwerveController;
import swervelib.SwerveDrive;
import swervelib.SwerveDriveTest;
import swervelib.SwerveModule;
import swervelib.imu.SwerveIMU;
import swervelib.math.SwerveMath;
import swervelib.motors.SwerveMotor;
import swervelib.parser.SwerveControllerConfiguration;
import swervelib.parser.SwerveDriveConfiguration;
import swervelib.parser.SwerveParser;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;

public class Swerve extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  public double maximumSpeed = Units.feetToMeters(14.5);
  private Rotation2d currentTargetAngle = new Rotation2d();
  private Pose2d allianceRelPose = new Pose2d();
  private boolean hadbadreading;

  /** Creates a new Swerve. */
  public Swerve(File config_dir) {

        // Configure the Telemetry before creating the SwerveDrive to avoid unnecessary
        // objects being created.
        SwerveDriveTelemetry.verbosity = Robot.isReal() ? TelemetryVerbosity.LOW : TelemetryVerbosity.HIGH;
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

            for(int i = 0; i < swerveDrive.getModules().length; i++) {
            // Lower the kS to reduce wobble?
            swerveDrive.getModules()[i].setFeedforward(new SimpleMotorFeedforward(SwerveConstants.MODULE_CONSTANTS[i].drive_kS * 0.1, SwerveConstants.MODULE_CONSTANTS[i].drive_kV, SwerveConstants.MODULE_CONSTANTS[i].drive_kA));
          }
        setupPathPlanner();
  }

        /**
     * Setup AutoBuilder for PathPlanner.
     */
    public void setupPathPlanner() {
      AutoBuilder.configureHolonomic(
              swerveDrive::getPose, // Robot pose supplier
              swerveDrive::resetOdometry, // Method to reset odometry (will be called if your auto has a starting pose)
              swerveDrive::getRobotVelocity, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
              swerveDrive::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
              new HolonomicPathFollowerConfig( // HolonomicPathFollowerConfig, this should likely live in your
                                               // Constants class
                      new PIDConstants(0.65, 0.0, 0.0),
                      // Translation PID constants
                      new PIDConstants(2,
                              0,
                              0.05), // TODO: TUNE THIS, NOT SURE IF SHOULD BE 0.005 PER OLD COMMENT
                      // Rotation PID constants
                      4.5,
                      // Max module speed, in m/s
                      swerveDrive.swerveDriveConfiguration.getDriveBaseRadiusMeters(),
                      // Drive base radius in meters. Distance from robot center to furthest module.
                      new ReplanningConfig()
              // Default path replanning config. See the API for the options here
              ),
              () -> {
                  // Boolean supplier that controls when the path will be mirrored for the red
                  // alliance
                  // This will flip the path being followed to the red side of the field.
                  // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                  var alliance = DriverStation.getAlliance();
                  return alliance.isPresent() && (alliance.get() == DriverStation.Alliance.Red);
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
          System.out.println("No alliance, setting to blue");
        }
        return currentAlliance;
      }

      /** Uses the current field rel pose to get the alliance rel pose, should be called periodically */
    public void updateAllianceRelPose() {
      Pose2d fieldRelPose = swerveDrive.getPose();
      allianceRelPose = alliancePoseFlipper(fieldRelPose);
  }

  /** Flips a Pose2d by 180 degrees if necessary */
    public Pose2d alliancePoseFlipper(Pose2d input) {
        if(getAllianceDefaultBlue() == Alliance.Blue) {
            return input;
        } else {
            return new Pose2d(FieldConstants.fieldLength - input.getX(), FieldConstants.fieldWidth - input.getY(), input.getRotation().minus(new Rotation2d(Math.PI)));
        }
    }

    /** Takes a chassis speeds object and flips it 180 degrees if needed*/
    public ChassisSpeeds allianceTargetSpeedsFlipper(ChassisSpeeds input) {
        if(getAllianceDefaultBlue() == Alliance.Blue) {
            return input;
        } else {
            return new ChassisSpeeds(-input.vxMetersPerSecond, -input.vyMetersPerSecond, input.omegaRadiansPerSecond);
        }
    }

    /** Takes a rotation2d and flips it 180 degrees */
    public Rotation2d allianceRotationFlipper(Rotation2d input) {
      return getAllianceDefaultBlue() == Alliance.Blue ? input : input.minus(new Rotation2d(Math.PI));
  }

  /** Gets current pose, alliance relative */
  public Pose2d getAllianceRelPose() {
      return allianceRelPose;
  }

    /** Drive robot with alliance relative speeds.
     *  Converts them to field relative speeds first then drives the robot.
     *  If using heading drive, it does not use the target angle at all.
    */
    public void driveAllianceRelative(double x, double y, double rotationRate, boolean headingDrive) {
        if (headingDrive) {
            swerveDrive.driveFieldOriented(swerveDrive.swerveController.getTargetSpeeds(getAllianceDefaultBlue() == Alliance.Blue ? x : -x, getAllianceDefaultBlue() == Alliance.Blue ? y : -y,
                    currentTargetAngle.getRadians(), swerveDrive.getYaw().getRadians(), swerveDrive.getMaximumVelocity()));
        } else {
            swerveDrive.drive(new Translation2d(getAllianceDefaultBlue() == Alliance.Blue ? x : -x, getAllianceDefaultBlue() == Alliance.Blue ? y : -y),
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
      swerveDrive.resetOdometry(new Pose2d(swerveDrive.getPose().getTranslation(), allianceRotationFlipper(new Rotation2d())));
  }

  /**
     * Sets the current angle of the gyro (Field Relative). If the robot reaches the same angle, the
     * gyro will report this angle.
     * 
     * @param currentAngle The angle that the gyro should read in its current state.
     */
    public void setGyroAngle(Rotation2d currentAngle) {
        swerveDrive.setGyro(new Rotation3d(0, 0, currentAngle.getRadians()));
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
                setTargetAllianceRelAngle(Rotation2d.fromRadians(Math.atan2(deadbandRotationInputs[1], deadbandRotationInputs[0])));
            }
            if (leftRotationInput != 0 && rightRotationInput == 0) {
                swerveDrive.setHeadingCorrection(false);
                double leftRotationOutput = Math.pow(leftRotationInput, 3) * swerveDrive.getMaximumAngularVelocity() * 2;
                driveAllianceRelative(xInput * swerveDrive.getMaximumVelocity(), yInput * swerveDrive.getMaximumVelocity(), leftRotationOutput, false);
            }
            // If right trigger pressed, rotate left at a rate proportional to the right
            // trigger input
            else if (rightRotationInput != 0 && leftRotationInput == 0) {
                swerveDrive.setHeadingCorrection(false);
                double rightRotationOutput = -Math.pow(rightRotationInput, 3) * swerveDrive.getMaximumAngularVelocity()
                        * 2;
                driveAllianceRelative(xInput * swerveDrive.getMaximumVelocity(), yInput * swerveDrive.getMaximumVelocity(), rightRotationOutput, false);
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
                    driveAllianceRelative(xInput * swerveDrive.getMaximumVelocity(), yInput * swerveDrive.getMaximumVelocity(), 0, false);
                }
            }
        });
    }

    // * Adds vision measurement from vision object to swerve
    public void addVisionData(VisionData visionData) {
        Pose2d swervePose = swerveDrive.getPose();
        double previousx = swervePose.getX();
        double previousy = swervePose.getY();
        Rotation2d previousTheta = swervePose.getRotation();
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

        // Add Vision Measurement if it passes the checks, but without taking into account vision yaw.
        swerveDrive.addVisionMeasurement(new Pose2d(visionData.visionPose().getTranslation(), swerveDrive.getOdometryHeading()), visionData.time(),
                visionData.visionReliability());
        Pose2d newPose = swerveDrive.getPose();
        if (Double.isNaN(newPose.getX()) || Double.isNaN(newPose.getY())) {
            // hadbadreading = true;
            Pose2d pose = new Pose2d(previousx, previousy, previousTheta);
            swerveDrive.resetOdometry(pose);
            System.out.println("Vision pose was invalid and not caught");
        }
    }

    // Takes a point and returns the desired heading for the swerve to be pointing
    // at the given point using the curent pose
    private double getAngleToPoint(Pose2d targetPoint) {
        Pose2d currentPose = swerveDrive.getPose();
        double desired_heading_rad = Math.atan2(targetPoint.getY() - currentPose.getY(),
                targetPoint.getX() - currentPose.getX());
        return desired_heading_rad;
    }

    // needs to be called repeatedly
    public Command pointAtVisionTarget(Pose2d targetPoint) {
        return new InstantCommand(() -> {
            double desired_heading_deg = getAngleToPoint(targetPoint);
            Rotation2d desired_heading = Rotation2d.fromDegrees(desired_heading_deg);
            swerveDrive.setHeadingCorrection(true);
            setTargetAngle(desired_heading);
        });
    }
        /**
     * <h2>Vision Targeting</h2>
     * Drives field oriented with translation X, Y, and points at the given target
     * point
     * 
     * @param translationX Supplier of translation in X axis
     * @param translationY Supplier of translation in Y axis
     * @param targetPoint  Supplier of target point
     * @return A RunCommand that drives the swerve drive with given translation and
     *         rotation
     */
    public Command driveTranslationAndPointAtTarget(DoubleSupplier translationX, DoubleSupplier translationY,
            Pose2d targetPoint) {
        return run(() -> {
            double desiredHeadingRad = getAngleToPoint(
                    AllianceFlipUtil.apply(targetPoint));
            Rotation2d desired_heading = Rotation2d.fromRadians(desiredHeadingRad);
            swerveDrive.setHeadingCorrection(true);
            double rawXInput = translationX.getAsDouble();
            double rawYInput = translationY.getAsDouble();
            double[] scaledDeadbandTranslationInputs = AllDeadbands
                    .applyScaledSquaredCircularDeadband(new double[] { rawXInput, rawYInput }, 0.1);
            double xInput = scaledDeadbandTranslationInputs[0];
            double yInput = scaledDeadbandTranslationInputs[1];
            // Make the robot move
            setTargetAngle(desired_heading);
            driveAllianceRelative(xInput, yInput, desiredHeadingRad, true);
        });
    }

      /**
   * Command to characterize the robot drive motors using SysId
   *
   * @return SysId Drive Command
   */
  public Command sysIdDriveMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setDriveSysIdRoutine(
            new Config(),
            this, swerveDrive, 12),
        3.0, 5.0, 3.0); // TODO: Tweak (increase quasitimeout if possible) for running sysid characterization
  }

  /**
   * Command to characterize the robot angle motors using SysId
   *
   * @return SysId Angle Command
   */
  public Command sysIdAngleMotorCommand()
  {
    return SwerveDriveTest.generateSysIdCommand(
        SwerveDriveTest.setAngleSysIdRoutine(
            new Config(),
            this, swerveDrive),
        3.0, 5.0, 3.0); // TODO: Tweak (increase quasitimeout if possible) if needed for running sysid characterization
  }


    public boolean isvisionOk() {
        return !hadbadreading;
    }

    /**
     * Add a fake vision reading for testing purposes.
     */
    public void addFakeVisionReading() {
        swerveDrive.addVisionMeasurement(new Pose2d(3, 3, Rotation2d.fromDegrees(65)), Timer.getFPGATimestamp());
    }

    public void setSwerveOffsets() {
        Rotation2d[] currentOffsets = new Rotation2d[4];
        Rotation2d[] newOffsets = new Rotation2d[4];
        Rotation2d[] measuredPositions = new Rotation2d[4];
        AbsoluteEncoder[] encoders = new AbsoluteEncoder[4];
        for (int i = 0; i < 4; i++) {
            encoders[i] = (AbsoluteEncoder) swerveDrive.getModules()[i].getAbsoluteEncoder().getAbsoluteEncoder();
            currentOffsets[i] = Rotation2d.fromDegrees(encoders[i].getZeroOffset());
            measuredPositions[i] = Rotation2d.fromDegrees(encoders[i].getPosition());
            newOffsets[i] = currentOffsets[i].plus(measuredPositions[i]).plus(Rotation2d.fromDegrees(getAngleForModule(i)));
            encoders[i].setZeroOffset(MathUtil.inputModulus(newOffsets[i].getDegrees(), 0, 360));
            swerveDrive.getModules()[i].getAngleMotor().burnFlash();
        }
    }

    private double getAngleForModule(int moduleNumber) {
      return switch (moduleNumber) {
          case 0 -> -90;
          case 1 -> 0;
          case 2 -> -180;
          case 3 -> -270;
          default -> 0;
      };
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
    swerveDrive.updateOdometry();
    updateAllianceRelPose();
  }
}
