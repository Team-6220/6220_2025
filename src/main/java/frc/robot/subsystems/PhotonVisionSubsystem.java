package frc.robot.subsystems;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  private PhotonCamera[] cameras =
  {
    new PhotonCamera("Bottom_Right_Cam"), //Top Right USB
    new PhotonCamera("Right_Ardu_Cam") //Bottom Right USB
    // new PhotonCamera("Left_Ardu_Cam")
  };
  private static PhotonVisionSubsystem INSTANCE = null;
  private ArrayList<PhotonTrackedTarget> bestTarget;
  private PhotonTrackedTarget noErrorHopefully;
  private String tableKey = "Vision_";

  private HashMap<Integer, List<PhotonPipelineResult>> results;

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].setPipelineIndex(0);
    }
  
    VisionConstants.setTagXYHeightAngle();

    results = new HashMap<Integer, List<PhotonPipelineResult>>();
    for (int i = 0; i < cameras.length; i++) {
      if (!cameras[i].getAllUnreadResults().isEmpty()) {
        results.put(i, cameras[i].getAllUnreadResults());
      }
      else
      {
        results.put(i, null);
      }
    }

    bestTarget = new ArrayList<PhotonTrackedTarget>();
    noErrorHopefully = new PhotonTrackedTarget();
    bestTarget.add(noErrorHopefully);
    bestTarget.add(noErrorHopefully);
    bestTarget.add(noErrorHopefully);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < cameras.length; i++) {
      if (!cameras[i].getAllUnreadResults().isEmpty()) {
        results.put(i, cameras[i].getAllUnreadResults());
      }
      if (!results.isEmpty() && !results.get(i).isEmpty()) {
        bestTarget.remove(i);
        bestTarget.add(i, results.get(i).get(0).getBestTarget());
        if(bestTarget.get(i)!= null)
        {
          SmartDashboard.putNumber(tableKey + i + "id", bestTarget.get(i).fiducialId);
          SmartDashboard.putNumber(tableKey + i + "pitch", bestTarget.get(i).pitch);
          SmartDashboard.putNumber(tableKey + i + "yaw", bestTarget.get(i).yaw);
          SmartDashboard.putNumber(tableKey + i + "ambiguity", bestTarget.get(i).poseAmbiguity);
          SmartDashboard.putNumber(tableKey + i + "skew", bestTarget.get(i).skew);
        }
      }
    }

  }

  public PhotonCamera[] getCameras() {
    return cameras;
  }

  public HashMap<Integer, List<PhotonPipelineResult>> getResults() {
    return results;
  }

  public ArrayList<PhotonTrackedTarget> getBestTargets() {
    return bestTarget;
  }

  public static synchronized PhotonVisionSubsystem getInstance() {
    if (INSTANCE == null) {
        INSTANCE = new PhotonVisionSubsystem();
    }
    return INSTANCE;
  }
}