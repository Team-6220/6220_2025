// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PhotonVisionSubsystem;

import org.photonvision.targeting.PhotonTrackedTarget;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class photonAlignCmd extends Command {
  private Swerve s_Swerve;
  private PhotonVisionSubsystem s_Photon;
  private int cameraNum;
  private double offset;
  
  /** Creates a new photonAlign. */
  public photonAlignCmd(int cameraNum, Swerve s_Swerve) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Photon = PhotonVisionSubsystem.getInstance();
    this.s_Swerve = s_Swerve;
    addRequirements(s_Photon, s_Swerve);
    this.cameraNum = cameraNum;
    this.offset = 0.0;
  }

  public photonAlignCmd(int cameraNum, Swerve s_Swerve, double offset) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Photon = PhotonVisionSubsystem.getInstance();
    this.s_Swerve = s_Swerve;
    addRequirements(s_Photon, s_Swerve);
    this.cameraNum = cameraNum;
    this.offset = offset;
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.resetTurnController();
    s_Swerve.setXYGoal(s_Swerve.getTargetX(), s_Swerve.getTargetY());
    System.out.print("Photon vision cmd initilized");
    // offsetX = VisionConstants.aprilTagCoordsX[s_Photon.getBestTarget().get(cameraNum - 1).getFiducialId()] - PhotonVisionCalculations.estimateOpposite(s_Photon.getBestTarget().get(cameraNum).getFiducialId(), cameraNum);
    // offsetY = VisionConstants.aprilTagCoordsY[s_Photon.getBestTarget().get(cameraNum - 1).getFiducialId()] - PhotonVisionCalculations.estimateAdjacent(s_Photon.getBestTarget().get(cameraNum).getFiducialId(), cameraNum);
    VisionConstants.setTagXYHeightAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    System.out.print("Photon vision cmd running");
    if(!s_Photon.getResults().get(cameraNum).isEmpty()) 
    {
      double currentY = s_Swerve.calculateY();
      PhotonTrackedTarget bestTarget = s_Photon.getBestTargets().get(cameraNum);
      currentY -= offset;

      Translation2d targetPosition = new Translation2d(s_Swerve.calculateX(), currentY);
      SmartDashboard.putNumber("Calculated Position y", currentY);
      SmartDashboard.putNumber("Target Position X ", targetPosition.getX());
      SmartDashboard.putNumber("Target Position y", targetPosition.getY());
      
      
      // s_Swerve.drive(targetPosition, 0.0, false, false);
      // s_Swerve.setAutoTurnHeading(VisionConstants.aprilTagAngle[bestTarget.fiducialId - 1]);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PHOTON ENDED");
    s_Swerve.stopDriving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
