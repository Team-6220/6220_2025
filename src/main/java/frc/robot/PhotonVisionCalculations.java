// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.estimation.CameraTargetRelation;

/** Add your docs here. */
public class PhotonVisionCalculations {
    /* 
     * 
     * 
    */
    public static double estimateDistance (PhotonCamera camera, int tagID) {
        if (tagID > 22 || tagID < 1) return -1;
        
        double aprilTagHeightInches = VisionConstants.aprilTagHeightInches[tagID];
        // NetworkTable table = NetworkTableInstance.getDefault().getTable("photon");
        // NetworkTableEntry ty = table.getEntry("ty");
        double targetOffsetAngle_Vertical = camera.camToTargDist();
        double cameraHeight = 20; //TODO: https://discord.com/channels/270263988615380993/270264069234098179/1329262819685826593
        double cameraMountAngle = 45.0;
        double totalAngleToTarget_deg = targetOffsetAngle_Vertical + cameraMountAngle;
        double totalAngleToTarget_rad = (totalAngleToTarget_deg * Math.PI) / 180.0;
        
        double instance = (aprilTagHeightInches + cameraHeight) / Math.tan(totalAngleToTarget_rad);
        
        return instance;
    }


}
