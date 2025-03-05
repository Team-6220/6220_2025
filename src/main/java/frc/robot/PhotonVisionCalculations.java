package frc.robot;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.Constants.VisionConstants;



/** Add your docs here. */
public class PhotonVisionCalculations {
    private static PhotonVisionSubsystem s_Photon = PhotonVisionSubsystem.getInstance();

    public PhotonVisionCalculations() {}

    public static void initPhoton() {}

    public static double estimateDistance (int tagID, int cameraNum) {
        double aprilTagHeightInches = VisionConstants.aprilTagXYHeightAngle.get(tagID)[2];
        
        double cameraHeight = VisionConstants.cameraSpecs.get(cameraNum)[0];
        double cameraMountAngle = VisionConstants.cameraSpecs.get(cameraNum)[1];

        double cameraOffset = s_Photon.getBestTarget().get(cameraNum).getPitch();
        
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
    
    // top - Processor and Coral Station
    // bot - Reef
    */
}