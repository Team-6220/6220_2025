// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PhotonVisionCalculations;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PhotonVisionSubsystem;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class photonAlignCmd extends Command {
  private Swerve s_Swerve;
  private PhotonVisionSubsystem s_Photon;
  private PhotonCamera camera;
  private double offsetX, offsetY;
  private int cameraNum;
  
  /** Creates a new photonAlign. */
  public photonAlignCmd(int cameraNum) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Photon = PhotonVisionSubsystem.getInstance();
    addRequirements(s_Photon);
    this.cameraNum = cameraNum;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.resetTurnController();
    s_Swerve.alignXYYaw(s_Swerve.getTargetX(), s_Swerve.getTargetY());
    offsetX = VisionConstants.aprilTagXYHeightAngle.get(s_Photon.getBestTarget().get(cameraNum).getFiducialId())[0] - PhotonVisionCalculations.estimateOpposite(s_Photon.getBestTarget().get(cameraNum).getFiducialId(), cameraNum);
    offsetY = VisionConstants.aprilTagXYHeightAngle.get(s_Photon.getBestTarget().get(cameraNum).getFiducialId())[1] - PhotonVisionCalculations.estimateAdjacent(s_Photon.getBestTarget().get(cameraNum).getFiducialId(), cameraNum);
    VisionConstants.setTagXYHeightAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(!s_Photon.getResults().get(cameraNum).isEmpty()) 
    {
      PhotonTrackedTarget bestTarget = s_Photon.getBestTarget().get(cameraNum);
      Translation2d targetPosition = new Translation2d(PhotonVisionCalculations.estimateAdjacent(bestTarget.fiducialId, cameraNum), PhotonVisionCalculations.estimateOpposite(bestTarget.fiducialId, cameraNum));
      //s_Swerve.get;
      //s_Swerve.drive(s_Swerve.getPidX().calculate(getcurrentPose() - xPidstart), swervesub.getypidspeed, swervesub.getyawpidspeed);
      s_Swerve.drive(targetPosition, 0.0, false, false);
      s_Swerve.setAutoTurnHeading(VisionConstants.aprilTagXYHeightAngle.get(bestTarget.fiducialId)[3]);
    }
    else {
      end(true);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.stopDriving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Swerve.getPidAtGoalX() && s_Swerve.getPidAtGoalY() && s_Swerve.getPidAtGoalYaw();
  }
}
