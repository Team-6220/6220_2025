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
    public static PhotonCamera[] cameras = {new PhotonCamera("limelight")};



    public static void initPhoton() {

    }
    public static double estimateDistance (int cameraID, int tagID) {
        if (tagID > 22 || tagID < 1) {
            return -1;
        }
        CameraTargetRelation relation = new CameraTargetRelation(null, null);
        double aprilTagHeightInches = VisionConstants.aprilTagHeightInches[tagID];
        NetworkTable table = NetworkTableInstance.getDefault().getTable("X");
        
    
        double cameraHeight = 20; //TODO: https://discord.com/channels/270263988615380993/270264069234098179/1329262819685826593
        double cameraMountAngle = 45.0;
        double totalAngleToTarget_deg = x + cameraMountAngle;
        double totalAngleToTarget_rad = (totalAngleToTarget_deg * Math.PI) / 180.0;
        
        double instance = (aprilTagHeightInches + cameraHeight) / Math.tan(totalAngleToTarget_rad);
        
        return instance;
    }

    public static double getXY() {
        
        return 1.0;
    }
}
