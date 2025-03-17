package frc.robot;
import frc.robot.subsystems.PhotonVisionSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.VisionConstants;



/** Add your docs here. */
public class PhotonVisionCalculations {
    private static PhotonVisionSubsystem s_Photon = PhotonVisionSubsystem.getInstance();

    public PhotonVisionCalculations() {}

    public static void initPhoton() {}

    //returns distance in meters
    // public static double estimateDistance (int tagID, int cameraNum) {
    //     if(tagID <= 0)
    //     {
    //         return 0.0;
    //     }
    //     double aprilTagHeightInches = VisionConstants.aprilTagHeightInches[tagID - 1];
        
    //     double cameraHeight = VisionConstants.cameraHeight[cameraNum];
    //     double cameraMountAngle = VisionConstants.cameraAngles[cameraNum];

    //     double cameraOffset = s_Photon.getBestTargets().get(cameraNum).getPitch();
        
    //     double totalAngleToTarget_deg = cameraOffset + cameraMountAngle;
    //     double totalAngleToTarget_rad = (totalAngleToTarget_deg * Math.PI) / 180.0;
        
    //     double instance = (aprilTagHeightInches + cameraHeight) / Math.tan(totalAngleToTarget_rad);

    //     // double instance = 0.1524 / Math.tan(Math.toRadians(s_Photon.getBestTargets().get(cameraNum).getPitch()));
        
    //     return instance * 0.0254;

    // }

    // public static double estimateOpposite(int tagID, int cameraNum) {
    //     if(tagID <= 0)
    //     {
    //         return 0.0;
    //     }
    //     double hypo = estimateDistance(tagID, cameraNum);
    //     double yaw = s_Photon.getBestTargets().get(cameraNum).getYaw();
    //     double instance = hypo * Math.sin(yaw);

    //     return instance;
    // }

    // public static double estimateAdjacent(int tagID, int cameraNum) {
    //     if(tagID <= 0)
    //     {
    //         System.out.println("sys id < 0");
    //         return 0.0;
    //     }
    //     double hypo = estimateDistance(tagID, cameraNum);
    //     double yaw = s_Photon.getBestTargets().get(cameraNum).getYaw();
    //     SmartDashboard.putNumber("vision calc yaw", yaw);
    //     double instance = hypo * Math.cos(yaw);

    //     return instance;
    // }
}