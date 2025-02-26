package frc.robot.subsystems;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;

public class PhotonVisionSubsystem extends SubsystemBase {
  private PhotonCamera[] cameras = new PhotonCamera[3];
  private static PhotonVisionSubsystem INSTANCE = null;

  public double offset;
  private List<PhotonPipelineResult> result;
  private PhotonTrackedTarget bestTarget;

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem() {
    cameras[0] = new PhotonCamera("Left_Ardu_Cam");
    cameras[1] = new PhotonCamera("Another_Ardu_Cam");
    cameras[2] = new PhotonCamera("Right_Ardu_Cam");
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].setPipelineIndex(0);
    }

    if (!cameras[0].getAllUnreadResults().isEmpty()) {
      result = cameras[0].getAllUnreadResults();
    }
    
    bestTarget = result.get(0).getBestTarget();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (!cameras[0].getAllUnreadResults().isEmpty() || !cameras[1].getAllUnreadResults().isEmpty() || !cameras[2].getAllUnreadResults().isEmpty()) {
      result = cameras[0].getAllUnreadResults();
      result.addAll(cameras[1].getAllUnreadResults());
      result.addAll(cameras[2].getAllUnreadResults());
    }
  }

  public PhotonCamera[] getCameras() {
    return cameras;
  }

  public List<PhotonPipelineResult> getResults() {
    return result;
  }

  public static synchronized PhotonVisionSubsystem getInstance() {
    if (INSTANCE == null) {
        INSTANCE = new PhotonVisionSubsystem();
    }
    return INSTANCE;
  }
}