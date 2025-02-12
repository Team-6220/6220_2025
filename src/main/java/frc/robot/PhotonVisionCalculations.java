// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
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
    private static PhotonCamera[] cameras;
    private static NetworkTable table;
    
    public PhotonVisionCalculations() {
        cameras = new PhotonCamera[3];
        for (int i = 0; i < cameras.length; i++) {
            cameras[i].setPipelineIndex(0);
        }
    }


    public static void initPhoton() {
        
    }
    public static double estimateDistance (int tagID) {
        
        double aprilTagHeightInches = VisionConstants.aprilTagHeightInches[tagID - 1];
        table = NetworkTableInstance.getDefault().getTable("X");
        NetworkTableEntry ty = table.getEntry("X");
        double cameraOffset = ty.getDouble(0.0);
        double cameraHeight = 20; //TODO: https://discord.com/channels/270263988615380993/270264069234098179/1329262819685826593
        double cameraMountAngle = 45.0;
        double totalAngleToTarget_deg = cameraOffset + cameraMountAngle;
        double totalAngleToTarget_rad = (totalAngleToTarget_deg * Math.PI) / 180.0;
        
        double instance = (aprilTagHeightInches + cameraHeight) / Math.tan(totalAngleToTarget_rad);
        
        return instance;
    }

    public static double estimateOpposite(int tagID) {
        double hypo = estimateDistance(tagID);
        double yaw = getYaw();

        double instance = hypo * Math.sin(yaw);

        return instance;
    }

    public static double estimateAdjacent(int tagID) {
        double hypo = estimateDistance(tagID);
        double yaw = getYaw();
        
        double instance = hypo * Math.cos(yaw);

        return instance;
    }

    public static double getYaw() {
        table = NetworkTableInstance.getDefault().getTable("Yaw");
        NetworkTableEntry ty = table.getEntry("Yaw");
        double yaw = ty.getDouble(0.0);
        return yaw;
    }

    public static double getXY() {
        
        return 1.0;
    }
}
