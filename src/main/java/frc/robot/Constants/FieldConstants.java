// Thank You 6328 Mechanical Advantage for the field constants!!!
package frc.robot.Constants;

import static edu.wpi.first.apriltag.AprilTagFields.k2025ReefscapeWelded;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.lib.util.LimelightHelpers;

import java.io.IOException;
import java.util.HashMap;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets of corners start in the lower left moving clockwise. 
 * All units in meters.
 
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE ALLIANCE wall.
 * 
 * Length refers to the x direction (as described by wpilib)
 * Width refers to the y direction (as described by wpilib)
 */
public class FieldConstants {
  public static double fieldLength = Units.inchesToMeters(690.876);
  public static double fieldWidth = Units.inchesToMeters(317);

  public static double reefLocationBackupDistance = Units.inchesToMeters(16);
  public static double reefLocationLeftDistance = Units.inchesToMeters(12);
  public static double reefLocationRightDistance = Units.inchesToMeters(-4);
  
  public static double startingLineX = Units.inchesToMeters(311.5);

  // /** Staging locations for each note */
  // public static final class StagingLocations {
  //   public static double centerlineX = fieldLength / 2.0;

  //   // need to update
  //   public static double centerlineFirstY = Units.inchesToMeters(29.638);
  //   public static double centerlineSeparationY = Units.inchesToMeters(66);
  //   public static double spikeX = Units.inchesToMeters(114);
  //   // need
  //   public static double spikeFirstY = Units.inchesToMeters(161.638);
  //   public static double spikeSeparationY = Units.inchesToMeters(57);

  //   public static Translation2d[] centerlineTranslations = new Translation2d[5];
  //   public static Translation2d[] spikeTranslations = new Translation2d[3];



  //   static {
  //     for (int i = 0; i < centerlineTranslations.length; i++) {
  //       centerlineTranslations[i] = new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
  //     }
  //   }

  //   static {
  //     for (int i = 0; i < spikeTranslations.length; i++) {
  //       spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
  //     }
  //   }
  // }

  public static double aprilTagWidth = Units.inchesToMeters(6.50);
  public static AprilTagFieldLayout aprilTags;

  // /** Each corner of the speaker * */
  // public static final class Speaker {

  //   // corners (blue alliance origin)
  //   public static Translation3d topRightSpeaker = new Translation3d(
  //       Units.inchesToMeters(18.055),
  //       Units.inchesToMeters(238.815),
  //       Units.inchesToMeters(83.091));

  //   public static Translation3d topLeftSpeaker = new Translation3d(
  //       Units.inchesToMeters(18.055),
  //       Units.inchesToMeters(197.765),
  //       Units.inchesToMeters(83.091));

  //   public static Translation3d bottomRightSpeaker = new Translation3d(0.0, Units.inchesToMeters(238.815),
  //       Units.inchesToMeters(78.324));
  //   public static Translation3d bottomLeftSpeaker = new Translation3d(0.0, Units.inchesToMeters(197.765),
  //       Units.inchesToMeters(78.324));

  //   /** Center of the speaker opening (blue alliance) */
  //   public static Pose2d centerSpeakerOpening = new Pose2d(
  //       bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5).toTranslation2d(), new Rotation2d());
  //   public static Pose2d centerSpeakerOpeningZeroX = new Pose2d(0, centerSpeakerOpening.getY(), new Rotation2d());

  // }

  public static Rotation2d coralStationBottomRot = new Rotation2d(45);
  public static Pose2d coralStationBottomPos = new Pose2d(1.139, 1.154, coralStationBottomRot);
  public static Rotation2d coralStationTopRot = new Rotation2d(-45);
  public static Pose2d coralStationTopPos = new Pose2d(1.295, 7, coralStationBottomRot);

  public static Rotation2d algaeProcessorRot = new Rotation2d(-90);
  public static Pose2d algaeProcessorPos = new Pose2d(5.988, 0.41, algaeProcessorRot);

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}