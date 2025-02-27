// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.lib.util;

import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants.FieldConstants;

/** Helper methods for the field :) */
public class FieldHelpers {
    private static AprilTagFieldLayout aprilTags = FieldConstants.aprilTags;

    /**
     * This hashmap maps tag numbers to the reef side they are on.<br>
     * </br>
     * key: tag number<br>
     * </br>
     * value: reef side number
     */
    public static HashMap<Integer, Integer> tagNoToReefSideRed = new HashMap<>() {
        {
            put(7, 1);
            put(8, 2);
            put(9, 3);
            put(10, 4);
            put(11, 5);
            put(6, 6);
        }
    };
    public static HashMap<Integer, Integer> tagNoToReefSideBlue = new HashMap<>() {
        {
            put(18, 1);
            put(17, 2);
            put(22, 3);
            put(21, 4);
            put(20, 5);
            put(19, 6);
        }
    };

    // Decides which alliance's apriltags to use.
    public static HashMap<Integer, Integer> tagNoToReefSide() {
        return DriverStation.getAlliance().orElseGet(() -> Alliance.Blue).equals(Alliance.Blue) ? tagNoToReefSideBlue
                : tagNoToReefSideRed;
    }

    /**
     * @param robotPose the pose of the robot
     * @return the reef side number that the robot is closest to
     */
    public static int findNearestReefSide(Pose2d robotPose) {
        Translation2d robotTranslation = robotPose.getTranslation();
        double minDistance = Double.MAX_VALUE;
        int nearestReefSide = -1;

        for (int tagID : tagNoToReefSide().keySet()) {
            Pose2d tagPose = aprilTags.getTagPose(tagID).get().toPose2d();
            Translation2d tagTranslation = tagPose.getTranslation();
            double distance = tagTranslation.getDistance(robotTranslation);
            if (distance < minDistance) {
                minDistance = distance;
                nearestReefSide = tagNoToReefSide().get(tagID);
            }
        }
        return nearestReefSide;
    }

    /**
     * Returns the pose that the robot should pathfind to for a particular reef side
     * on the left or right. Reef side can be returned by findNearestReefSide
     * 
     * @param reefSide    goes from 1 to 6, starting from the side closest to the
     *                    alliance station and going counterclockwise
     * @param isRightSide is true if we're on the right side of the specified reef
     *                    side
     * @return a Pose2d representing the location and orientation of the robot if
     *         facing the reef on the specified
     */
    public static Pose2d reefLocation(int reefSide, BooleanSupplier isRightSideSupplier) {

        int poseCode = (1 <= reefSide && reefSide <= 6) ? (reefSide * 2 + (isRightSideSupplier.getAsBoolean() ? 1 : 0))
                : -1;
        Rotation2d angle = Rotation2d.fromDegrees(switch (reefSide) {
            case 1 -> 0;
            case 2 -> 60;
            case 3 -> 120;
            case 4 -> 180;
            case 5 -> 240;
            case 6 -> 300;
            default -> 0;
        });

        double[] pos = switch (poseCode) {
            case 2 -> new double[] { 158.00, 164.94 };
            case 3 -> new double[] { 158.00, 152.06 };
            case 4 -> new double[] { 168.80, 133.36 };
            case 5 -> new double[] { 179.95, 126.92 };
            case 6 -> new double[] { 201.55, 126.92 };
            case 7 -> new double[] { 212.70, 133.36 };
            case 8 -> new double[] { 223.50, 152.06 };
            case 9 -> new double[] { 223.50, 164.94 };
            case 10 -> new double[] { 212.70, 183.64 };
            case 11 -> new double[] { 201.55, 190.08 };
            case 12 -> new double[] { 179.95, 190.08 };
            case 13 -> new double[] { 168.80, 183.64 };
            default -> new double[] { 0, 0 };
        };

        Pose2d pose = new Pose2d(Units.inchesToMeters(pos[0]), Units.inchesToMeters(pos[1]), angle);

        // back up pose by 16" so it's not overlapping the reef
        pose = pose.transformBy(
                new Transform2d(new Translation2d(-FieldConstants.reefLocationBackupDistance, 0), new Rotation2d()));

        return pose;
    }

    /**
     * Returns the pose that the robot should pathfind to for a particular reef side
     * on the left or right. Reef side can be returned by findNearestReefSide
     * 
     * @param robotPose    current pose of the robot, used to determine closest reef side
     * @param isRightSide is true if we're on the right side of the specified reef
     *                    side
     * @return a Pose2d representing the location and orientation of the robot if
     *         facing the reef on the specified
     */
    public static Pose2d reefLocation(Pose2d robotPose, BooleanSupplier isRightSideSupplier) {
        int reefSide = findNearestReefSide(robotPose);
        return reefLocation(reefSide, isRightSideSupplier);
    }
}
