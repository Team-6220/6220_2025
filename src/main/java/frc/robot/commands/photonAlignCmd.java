// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PhotonVisionSubsystem;

import org.photonvision.targeting.PhotonTrackedTarget;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class photonAlignCmd extends Command {
  private Swerve s_Swerve;
  private PhotonVisionSubsystem s_Photon;

  private final TunableNumber xKP = new TunableNumber("x kP", Constants.SwerveConstants.xKP);
  private final TunableNumber xKI = new TunableNumber("x kI", Constants.SwerveConstants.xKI);
  private final TunableNumber xKD = new TunableNumber("x kD", Constants.SwerveConstants.xKD);
  private final TunableNumber xMaxVel = new TunableNumber("x MaxVel", Constants.SwerveConstants.xMaxVel);
  private final TunableNumber xMaxAccel = new TunableNumber("x Accel", Constants.SwerveConstants.xMaxAccel);

  private final TunableNumber yKP = new TunableNumber("y kP", Constants.SwerveConstants.yKP);
  private final TunableNumber yKI = new TunableNumber("y kI", Constants.SwerveConstants.yKI);
  private final TunableNumber yKD = new TunableNumber("y kD", Constants.SwerveConstants.yKD);
  private final TunableNumber yMaxVel = new TunableNumber("y MaxVel", Constants.SwerveConstants.yMaxVel);
  private final TunableNumber yMaxAccel = new TunableNumber("y Accel", Constants.SwerveConstants.yMaxAccel);
  private int cameraNum;
  private double xSetpoint, ySetpoint;
  private PIDController xcontroller = new PIDController(xKP.get(), xKI.get(), xKD.get());
  private PIDController ycontroller = new PIDController(yKP.get(), yKI.get(), yKD.get());
  // private PhotonTrackedTarget bestTarget;
  
  /** Creates a new photonAlign. */
  public photonAlignCmd(int cameraNum, Swerve s_Swerve, double xSetpoint, double ySetpoint)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Photon = PhotonVisionSubsystem.getInstance();
    this.s_Swerve = s_Swerve;
    addRequirements(s_Photon, s_Swerve);
    this.cameraNum = cameraNum;
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;
  }

  public photonAlignCmd(int cameraNum, Swerve s_Swerve, double offset) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Photon = PhotonVisionSubsystem.getInstance();
    this.s_Swerve = s_Swerve;
    addRequirements(s_Photon, s_Swerve);
    this.cameraNum = cameraNum;
  } 

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.resetTurnController();
    // s_Swerve.setXYGoal(s_Swerve.getTargetX(), s_Swerve.getTargetY());
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
      PhotonTrackedTarget bestTarget = s_Photon.getBestTargets().get(cameraNum);
      if(bestTarget != null)
      {
        Transform3d currentPose = bestTarget.getBestCameraToTarget();
        
        xcontroller.setSetpoint(xSetpoint);      
        ycontroller.setSetpoint(ySetpoint);
        double xout = xcontroller.calculate(currentPose.getX());
        double yout = ycontroller.calculate(currentPose.getY());
        double thetaout = s_Swerve.getTurnPidSpeed();
        SmartDashboard.putNumber("x pid out", xout);
        SmartDashboard.putNumber("y pid out", yout);
        SmartDashboard.putNumber("theta pid out", thetaout);
        s_Swerve.setAutoTurnHeading(VisionConstants.aprilTagAngle[bestTarget.getFiducialId()-1]);
        s_Swerve.drive(new Translation2d(-xout, -yout), -thetaout, false, false);
        // s_Swerve.setAutoTurnHeading(VisionConstants.aprilTagAngle[bestTarget.fiducialId - 1]);
      }
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
    return s_Photon.getResults().get(cameraNum).isEmpty();//if there's no tag automatically stop it from driving
  }
}
