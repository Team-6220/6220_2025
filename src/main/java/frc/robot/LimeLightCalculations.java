<<<<<<< Updated upstream
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.LimelightHelpers.LimelightResults;
import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.Constants.VisionConstants;

/** Add your docs here. */
public class LimeLightCalculations {
    public static void estimateDistance (int tagID) {
        if (tagID > 22 || tagID < 1) return;
        double aprilTagHeightInches = VisionConstants.aprilTagHeightInches[tagID];
        double targetOffsetAngle_Vertical = ty.getDouble(0.0);
        double cameraHeight; //TODO: https://discord.com/channels/270263988615380993/270264069234098179/1329262819685826593
        double cameraMountAngle = 45.0;
        double totalAngleToTarget_deg = targetOffsetAngle_Vertical + cameraMountAngle;
        double totalAngleToTarget_rad = (totalAngleToTarget_deg * Math.PI) / 180.0;
        
        double istance = (aprilTagHeightInches + cameraHeight) / M
        }

    public static Pose2d updateVisionRobotPose2d(LimelightResults limelightResults, SwerveDrivePoseEstimator swerveDrivePoseEstimator, double constant) {

        return null;
    }
}
=======
// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;
// import frc.robot.LimelightHelpers.LimelightResults;
// import frc.robot.LimelightHelpers.LimelightTarget_Fiducial;
// import frc.robot.subsystems.Swerve;
// import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import frc.robot.Constants.VisionConstants;

// /** Add your docs here. */
// public class LimeLightCalculations {
    

//     /** Returns distance from tagID
//      * @param tagID
//      * April tagID
//      * */
//     public static double estimateDistance (int tagID) {
//         NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
//         NetworkTableEntry ty = table.getEntry("ty");

//         if (tagID > 22 || tagID < 1) return 0.0;
//         double aprilTagHeightInches = VisionConstants.aprilTagHeightInches[tagID];
//         double targetOffsetAngle_Vertical = ty.getDouble(0.0);
//         double totalAngleToTarget_deg = targetOffsetAngle_Vertical + VisionConstants.cameraMountAngle;
//         double totalAngleToTarget_rad = (totalAngleToTarget_deg * Math.PI) / 180.0;
        
//         return (aprilTagHeightInches - VisionConstants.cameraMountHeight) / Math.tan(totalAngleToTarget_rad); //returns distance;
//         }

//     public static Pose2d updateVisionRobotPose2d(LimelightResults limelightResults, SwerveDrivePoseEstimator swerveDrivePoseEstimator, double constant) {

//         return null;
//     }
    
// }
>>>>>>> Stashed changes
