// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import org.photonvision.PhotonCamera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class PhotonVisionSubsystem extends SubsystemBase {
  private PhotonCamera[] cameras = new PhotonCamera[3];
  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem() {
    cameras[0] = new PhotonCamera("");
    cameras[1] = new PhotonCamera("");
    cameras[2] = new PhotonCamera("");
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].setPipelineIndex(0);
    }
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
