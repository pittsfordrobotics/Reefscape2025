package frc.robot.subsystems.Vision;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.lib.util.LimelightHelpers;
import frc.robot.lib.util.LimelightHelpers.PoseEstimate;
import frc.robot.lib.util.LimelightHelpers.RawFiducial;
import edu.wpi.first.wpilibj.Timer;

public class VisionIOLimelight implements VisionIO {

    private final String cameraName;

    public VisionIOLimelight(String cameraName) {
        this.cameraName = cameraName;
        setLEDs(LED.OFF, cameraName);
        setPipeline(Pipelines.Test);
    }

    // Uses limelight lib and network tables to get the values from the limelight
    // TODO: Use getBotPoseEstimate() from LimelightHelpers
    @Override
    public void updateInputs(VisionIOInputs inputs, double gyroAngle) {
        LimelightHelpers.SetRobotOrientation(cameraName, gyroAngle, 0, 0, 0, 0, 0 );
        // Gets the needed data from the networktables
        PoseEstimate botPoseEstimate = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(cameraName);

        // Latency (Pipeline + Capture)
        double latency = botPoseEstimate.latency;
        inputs.totalLatency = latency; // ms

        // Timestamp Needs to be the same as everything else
        // Dont know how this built in one functions
        // inputs.captureTimestamp = botPoseEstimate.timestampSeconds; // dont want to risk it
        inputs.captureTimestamp = Timer.getFPGATimestamp() - Units.millisecondsToSeconds(latency);
        
        // Avg Distance, Tag Count, and Pose
        inputs.avgTagDist = botPoseEstimate.avgTagDist; // meters
        inputs.tagCount = botPoseEstimate.tagCount; // int
        inputs.pose = botPoseEstimate.pose; // Pose2d

        // Updates TagIDs and TagDistances using the per tag data (RawFiducial[])
        RawFiducial[] fiducials = botPoseEstimate.rawFiducials;
        int[] tagIDDs = new int[fiducials.length];
        double[] tagDistances = new double[fiducials.length];
        for (int i = 0; i < fiducials.length; i++) {
            tagIDDs[i] = fiducials[i].id;
            tagDistances[i] = fiducials[i].distToRobot;
        }
        inputs.tagIDs = tagIDDs;
        inputs.tagDistances = tagDistances;

        // Direct access to the network tables for other values
        final NetworkTable limelight = LimelightHelpers.getLimelightNTTable(cameraName);
        // connected if heartbeat value is not zero
        NetworkTableEntry heartbeatEntry = limelight.getEntry("hb");
        inputs.connected = heartbeatEntry.getDouble(0.0) > 0.0;
        // has target if true
        inputs.hasTarget = LimelightHelpers.getTV(cameraName);
    }

    @Override
    public void setPipeline(Pipelines pipeline) {
        NetworkTable limelight = LimelightHelpers.getLimelightNTTable(cameraName);
        limelight.getEntry("pipeline").setDouble(pipeline.getNum());
    }

    @Override
    public void setCameraModes(CameraMode camera) {
        final NetworkTable limelight = LimelightHelpers.getLimelightNTTable(cameraName);
        limelight.getEntry("camMode").setDouble(camera.getNum());
    }

    private void setLEDs(LED led, String limelightName) {
        final NetworkTable limelight = LimelightHelpers.getLimelightNTTable(limelightName);
        limelight.getEntry("ledMode").setDouble(led.getNum());
    }
}