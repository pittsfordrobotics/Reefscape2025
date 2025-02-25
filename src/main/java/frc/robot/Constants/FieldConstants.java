// Thank You 6328 Mechanical Advantage for the field constants!!!
package frc.robot.Constants;

import static edu.wpi.first.apriltag.AprilTagFields.k2025ReefscapeWelded;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.lib.util.LimelightHelpers;

import java.io.IOException;
import java.util.HashMap;

/**
 * Contains various field dimensions and useful reference points. Dimensions are
 * in meters, and sets
 * of corners start in the lower left moving clockwise. <b>All units in
 * Meters</b> <br>
 * <br>
 *
 * <p>
 * All translations and poses are stored with the origin at the rightmost point
 * on the BLUE
 * ALLIANCE wall.<br>
 * <br>
 * Length refers to the <i>x</i> direction (as described by wpilib) <br>
 * Width refers to the <i>y</i> direction (as described by wpilib)
 */
public class FieldConstants {
  public static double fieldLength = Units.inchesToMeters(690.876);
  public static double fieldWidth = Units.inchesToMeters(317);

  public static double reefLocationBackupDistance = Units.inchesToMeters(16);

  /** This hashmap maps tag numbers to the reef side they are on.<br></br>
   * key: tag number<br></br>
   * value: reef side number
   */
  public static HashMap<Integer, Integer> tagNoToReefSide = new HashMap<>() {{
    put(7, 1);   put(18, 1);
    put(8, 2);   put(17, 2);
    put(9, 3);   put(22, 3);
    put(10, 4);  put(21, 4);
    put(11, 5);  put(20, 5);
    put(6, 6);   put(19, 6);
  }};

  /** 
   * @param robotPose the pose of the robot
   * @return the reef side number that the robot is closest to
   */
  public static int findNearestReefSide(Pose2d robotPose) {
    Translation2d robotTranslation = robotPose.getTranslation();
    double minDistance = Double.MAX_VALUE;
    int nearestReefSide = -1;

    for(int tagID : tagNoToReefSide.keySet()) {
      Pose2d tagPose = aprilTags.getTagPose(tagID).get().toPose2d();
      Translation2d tagTranslation = tagPose.getTranslation();
      double distance = tagTranslation.getDistance(robotTranslation);
      if(distance < minDistance) {
        minDistance = distance;
        nearestReefSide = tagNoToReefSide.get(tagID);
      }
    }
    return nearestReefSide;
  }
  
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

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}