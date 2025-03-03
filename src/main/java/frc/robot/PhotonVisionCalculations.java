// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleArrayTopic;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.CameraTargetRelation;

/** Add your docs here. */
public class PhotonVisionCalculations {
    private static PhotonVisionSubsystem s_Photon = PhotonVisionSubsystem.getInstance();
    private static PhotonCamera[] cameras = s_Photon.getCameras();

    public PhotonVisionCalculations() {}

    public static void initPhoton() {}

    public static double estimateDistance (int tagID, int cameraNum) {
        double aprilTagHeightInches = VisionConstants.aprilTagXYHeightAngle.get(tagID)[2];
        
        double cameraHeight = VisionConstants.cameraSpecs.get(cameraNum)[0];
        double cameraMountAngle = VisionConstants.cameraSpecs.get(cameraNum)[1];

        double cameraOffset = s_Photon.getResults().get(cameraNum).
        
        double totalAngleToTarget_deg = cameraOffset + cameraMountAngle;
        double totalAngleToTarget_rad = (totalAngleToTarget_deg * Math.PI) / 180.0;
        
        double instance = (aprilTagHeightInches + cameraHeight) / Math.tan(totalAngleToTarget_rad);
        
        return instance;
    }

    public static double estimateOpposite(int tagID, int cameraNum) {
        double hypo = estimateDistance(tagID, cameraNum);
        double yaw = s_Photon.getBestTarget().get(cameraNum).getYaw();

        double instance = hypo * Math.sin(yaw);

        return instance;
    }

    public static double estimateAdjacent(int tagID, int cameraNum) {
        double hypo = estimateDistance(tagID, cameraNum);
        double yaw = s_Photon.getBestTarget().get(cameraNum).getYaw();
        
        double instance = hypo * Math.cos(yaw);

        return instance;
    }
    /*
    // Arducam lens locations
    // Bottom right:
    // Z: 29.5” + 1.724” up
    // Y: 3.3” towards back
    // Angled 210° (down 30° from level)
    
    // Top right:
    // Z: 35.707” + 1.724” up
    // Y: 1.257” towards back
    // Angled 135° (45° up)
    
    // Top left:
    // Z: 35.707” + 1.724” up
    // Y: 0.157” towards front
    // Angled 45° (45° up)
    
    // top - Processor, Barge, and Coral Station
    // bot - Reef
    */
}