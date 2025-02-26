// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.PhotonVisionCalculations;
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
  private double offset;
  
  /** Creates a new photonAlign. */
  public photonAlignCmd() {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Photon = PhotonVisionSubsystem.getInstance();
    addRequirements(s_Photon);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.resetTurnController();
    s_Swerve.alignXYYaw(s_Swerve.getTargetX(), s_Swerve.getTargetY(), s_Swerve.getTargetYaw());
    offset = PhotonVisionCalculations.estimateDistance(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    if(!resultList.isEmpty())
    {
      PhotonTrackedTarget bestTarget = result.getBestTarget();
      Translation2d targetPosition = new Translation2d(PhotonVisionCalculations.estimateAdjacent(bestTarget.fiducialId), PhotonVisionCalculations.estimateOpposite(bestTarget.fiducialId));
      //s_Swerve.get;
      //s_Swerve.drive(s_Swerve.getPidX().calculate(getcurrentPose() - xPidstart), swervesub.getypidspeed, swervesub.getyawpidspeed);
      s_Swerve.drive(targetPosition, PhotonVisionCalculations.getYaw(), false, false);
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
