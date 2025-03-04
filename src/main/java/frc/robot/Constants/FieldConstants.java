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

  /** This hashmap maps tag numbers to the reef side they are on.
   * key: tag number
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
  public static double aprilTagWidth = Units.inchesToMeters(6.50);
  public static AprilTagFieldLayout aprilTags;

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2025ReefscapeWelded.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}