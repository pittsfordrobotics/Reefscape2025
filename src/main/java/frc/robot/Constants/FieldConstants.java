// Thank You 6328 Mechanical Advantage for the field constants!!!
package frc.robot.Constants;

import static edu.wpi.first.apriltag.AprilTagFields.k2025Reefscape;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;

import java.io.IOException;

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
  //most of this is out of date. todo: update everything
  public static double fieldLength = Units.inchesToMeters(690.876);
  public static double fieldWidth = Units.inchesToMeters(317);

  /**
   * Returns the pose that the robot should pathfind to for a particular reef side on the left or right
   * @param reefSide goes from 1 to 6, starting from the side closest to the alliance station and going counterclockwise
   * @param isRightSide is true if we're on the right side of the specified reef side
   * @return a Pose2d representing the location and orientation of the robot if facing the reef on the specified 
   */
  public static Pose2d reefLocation(int reefSide, boolean isRightSide) {

    int poseCode = (1 <= reefSide && reefSide <= 6) ? (reefSide * 2 + (isRightSide ? 1 : 0)) : -1;
    Rotation2d angle = Rotation2d.fromDegrees(switch(poseCode) {
      case 2, 3 -> 0;
      case 4, 5 -> 60;
      case 6, 7 -> 120;
      case 8, 9 -> 180;
      case 10, 11 -> 240;
      case 12, 13 -> 300;
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
    pose.transformBy(new Transform2d(new Translation2d(-Units.inchesToMeters(16), 0), new Rotation2d()));

    return pose;
  }

  public static double wingX = Units.inchesToMeters(229.201);
  public static double podiumX = Units.inchesToMeters(126.75);
  public static double startingLineX = Units.inchesToMeters(74.111);

  // these 3 are taken from the initial poses in pathplanner for the shoot and stay autos from each position
  // they may not be the same as the ones used in autos that do move, and they should not be used for those without checking first
  public static Pose2d ampsideStartPose = new Pose2d(0.753, 6.669, new Rotation2d(Math.toRadians(-120)));
  public static Pose2d middleStartPose = new Pose2d(1.343, 5.550, new Rotation2d(Math.toRadians(180)));
  public static Pose2d podiumsideStartPose = new Pose2d(0.760, 4.449, new Rotation2d(Math.toRadians(120)));

  public static Translation2d ampCenter = new Translation2d(Units.inchesToMeters(72.455),
      Units.inchesToMeters(322.996));

  public static Translation2d ampCenterRED_THISIFFORREDAMP = new Translation2d(fieldLength - Units.inchesToMeters(72.455),
      Units.inchesToMeters(322.996));

  /** Staging locations for each note */
  public static final class StagingLocations {
    public static double centerlineX = fieldLength / 2.0;

    // need to update
    public static double centerlineFirstY = Units.inchesToMeters(29.638);
    public static double centerlineSeparationY = Units.inchesToMeters(66);
    public static double spikeX = Units.inchesToMeters(114);
    // need
    public static double spikeFirstY = Units.inchesToMeters(161.638);
    public static double spikeSeparationY = Units.inchesToMeters(57);

    public static Translation2d[] centerlineTranslations = new Translation2d[5];
    public static Translation2d[] spikeTranslations = new Translation2d[3];



    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] = new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }

    static {
      for (int i = 0; i < spikeTranslations.length; i++) {
        spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
      }
    }
  }

  public static double aprilTagWidth = Units.inchesToMeters(6.50);
  public static AprilTagFieldLayout aprilTags;

  /** Each corner of the speaker * */
  public static final class Speaker {

    // corners (blue alliance origin)
    public static Translation3d topRightSpeaker = new Translation3d(
        Units.inchesToMeters(18.055),
        Units.inchesToMeters(238.815),
        Units.inchesToMeters(83.091));

    public static Translation3d topLeftSpeaker = new Translation3d(
        Units.inchesToMeters(18.055),
        Units.inchesToMeters(197.765),
        Units.inchesToMeters(83.091));

    public static Translation3d bottomRightSpeaker = new Translation3d(0.0, Units.inchesToMeters(238.815),
        Units.inchesToMeters(78.324));
    public static Translation3d bottomLeftSpeaker = new Translation3d(0.0, Units.inchesToMeters(197.765),
        Units.inchesToMeters(78.324));

    /** Center of the speaker opening (blue alliance) */
    public static Pose2d centerSpeakerOpening = new Pose2d(
        bottomLeftSpeaker.interpolate(topRightSpeaker, 0.5).toTranslation2d(), new Rotation2d());
    public static Pose2d centerSpeakerOpeningZeroX = new Pose2d(0, centerSpeakerOpening.getY(), new Rotation2d());

  }

  static {
    try {
      aprilTags = AprilTagFieldLayout.loadFromResource(k2025Reefscape.m_resourceFile);
    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }

  // public static Pose3d allianceFlipper(Pose3d pose, Alliance alliance) {
  //   if (alliance == Alliance.Blue) {
  //     return pose;
  //   }
  //   // Keep the x-component, change the magnitude of y component.
  //   // Height remains the same.
  //   Translation3d transformedTranslation = new Translation3d(pose.getTranslation().getX(),
  //       FieldConstants.fieldWidth - pose.getTranslation().getY(), pose.getTranslation().getZ());

  //   // Rotate by 180 degrees
  //   Rotation3d transformedHolonomicRotation = pose.getRotation().times(-1);
  //   return new Pose3d(transformedTranslation, transformedHolonomicRotation);
  // }
}