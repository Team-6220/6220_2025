// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
  private PhotonTrackedTarget resultList;

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem() {
    cameras[0] = new PhotonCamera("Left_Ardu_Cam");
    cameras[1] = new PhotonCamera("Another_Ardu_Cam");
    cameras[2] = new PhotonCamera("Right_Ardu_Cam");
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].setPipelineIndex(0);
    }
    result = cameras[0].getAllUnreadResults().isEmpty() ? new List<PhotonPipelineResult>(): cameras[0].getAllUnreadResults();
    resultList = result.get(0).getBestTarget();
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
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
