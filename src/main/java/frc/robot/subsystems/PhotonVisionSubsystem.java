package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  private PhotonCamera[] cameras = new PhotonCamera[3];
  private static PhotonVisionSubsystem INSTANCE = null;
  private ArrayList<PhotonTrackedTarget> bestTarget;

  private HashMap<Integer, List<PhotonPipelineResult>> results;

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem() {
    cameras[0] = new PhotonCamera("Left_Ardu_Cam");
    cameras[1] = new PhotonCamera("Another_Ardu_Cam");
    cameras[2] = new PhotonCamera("Right_Ardu_Cam");
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].setPipelineIndex(0);
    }
  
    VisionConstants.setTagXYHeightAngle();
    results = new HashMap<Integer, List<PhotonPipelineResult>>();

    results.put(0, cameras[0].getAllUnreadResults());
    results.put(1, cameras[1].getAllUnreadResults());
    results.put(2, cameras[2].getAllUnreadResults());

    bestTarget = new ArrayList<PhotonTrackedTarget>();
    bestTarget.add(results.get(0).get(0).getBestTarget());
    bestTarget.add(results.get(1).get(1).getBestTarget());
    bestTarget.add(results.get(2).get(2).getBestTarget());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!cameras[0].getAllUnreadResults().isEmpty() || !cameras[1].getAllUnreadResults().isEmpty() || !cameras[2].getAllUnreadResults().isEmpty()) {
      results.put(0, cameras[0].getAllUnreadResults());
      results.put(1, cameras[1].getAllUnreadResults());
      results.put(2, cameras[2].getAllUnreadResults());
    }
  }

  public PhotonCamera[] getCameras() {
    return cameras;
  }

  public HashMap<Integer, List<PhotonPipelineResult>> getResults() {
    return results;
  }

  public ArrayList<PhotonTrackedTarget> getBestTarget() {
    return bestTarget;
  }

  public static synchronized PhotonVisionSubsystem getInstance() {
    if (INSTANCE == null) {
        INSTANCE = new PhotonVisionSubsystem();
    }
    return INSTANCE;
  }
}