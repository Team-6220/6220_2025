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
import frc.robot.SwerveConstants;
import frc.robot.VisionConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.PhotonVisionSubsystem;

import java.util.List;

import org.photonvision.targeting.PhotonTrackedTarget;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class photonAlignCmd extends Command {
  private Swerve s_Swerve;
  private PhotonVisionSubsystem s_Photon;

  private final TunableNumber xKP = new TunableNumber("x kP", SwerveConstants.xKP);
  private final TunableNumber xKI = new TunableNumber("x kI", SwerveConstants.xKI);
  private final TunableNumber xKD = new TunableNumber("x kD", SwerveConstants.xKD);
  private final TunableNumber xMaxVel =
      new TunableNumber("x MaxVel", SwerveConstants.xMaxVel);
  private final TunableNumber xMaxAccel =
  new TunableNumber("x Accel", SwerveConstants.xMaxAccel);
  private final TunableNumber xTolerance = new TunableNumber("x Tolerance", SwerveConstants.xTolerance);

  private final TunableNumber yKP = new TunableNumber("y kP", SwerveConstants.yKP);
  private final TunableNumber yKI = new TunableNumber("y kI", SwerveConstants.yKI);
  private final TunableNumber yKD = new TunableNumber("y kD", SwerveConstants.yKD);
  private final TunableNumber yTolerance = new TunableNumber("y Tolerance", SwerveConstants.yTolerance);
  private final TunableNumber yMaxVel =
      new TunableNumber("y MaxVel", SwerveConstants.yMaxVel);
  private final TunableNumber yMaxAccel =
      new TunableNumber("y Accel", SwerveConstants.yMaxAccel);
  private int cameraNum;
  private double xSetpoint, ySetpoint;
  private int lockedFiducialID = -1;
  private PIDController xcontroller = new PIDController(xKP.get(), xKI.get(), xKD.get());
  private PIDController ycontroller = new PIDController(yKP.get(), yKI.get(), yKD.get());

  private boolean isFinished;
  //use this to count how long the entire thing ends itself
  //  if it simply didn't see anything but everything's functioning
  private int autoEndcount = 0;

  // private PhotonTrackedTarget bestTarget;

  /** Creates a new photonAlign. */
  public photonAlignCmd(int cameraNum, Swerve s_Swerve, double xSetpoint, double ySetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Photon = PhotonVisionSubsystem.getInstance(VisionConstants.cameraNames);
    this.s_Swerve = s_Swerve;
    addRequirements(s_Photon, s_Swerve);
    this.cameraNum = cameraNum;
    this.xSetpoint = xSetpoint;
    this.ySetpoint = ySetpoint;
  }

  // public photonAlignCmd(int cameraNum, Swerve s_Swerve, double offset) {
  //   // Use addRequirements() here to declare subsystem dependencies.
  //   s_Photon = PhotonVisionSubsystem.getInstance(VisionConstants.cameraNames);
  //   this.s_Swerve = s_Swerve;
  //   addRequirements(s_Photon, s_Swerve);
  //   this.cameraNum = cameraNum;
  // }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    s_Swerve.resetTurnController();
    // s_Swerve.setXYGoal(s_Swerve.getTargetX(), s_Swerve.getTargetY());
    System.out.print("Photon vision cmd initilized");
    // offsetX = VisionConstants.aprilTagCoordsX[s_Photon.getBestTarget().get(cameraNum -
    // 1).getFiducialId()] -
    // PhotonVisionCalculations.estimateOpposite(s_Photon.getBestTarget().get(cameraNum).getFiducialId(), cameraNum);
    // offsetY = VisionConstants.aprilTagCoordsY[s_Photon.getBestTarget().get(cameraNum -
    // 1).getFiducialId()] -
    // PhotonVisionCalculations.estimateAdjacent(s_Photon.getBestTarget().get(cameraNum).getFiducialId(), cameraNum);
    VisionConstants.setTagXYHeightAngle();

    // call initPhoton here so that it will declare objects when camera is plugged in while the code
    // is running
    // call initphoton also so that things will get cleared out if something disconnects
    s_Photon.initPhoton();
    isFinished = false;
    autoEndcount = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.print("Photon vision cmd running");
    s_Photon.updatePhoton();
    if(xTolerance.hasChanged())
    {
      xcontroller.setTolerance(xTolerance.get());
    }
    if(yTolerance.hasChanged())
    {
      ycontroller.setTolerance(yTolerance.get());
    }
    if (s_Photon.getResults().containsKey(cameraNum)
        && s_Photon.getResults().get(cameraNum) != null
        && !s_Photon.getResults().get(cameraNum).isEmpty()) {
      List<PhotonTrackedTarget> bestTarget = s_Photon.getBestTargets().get(cameraNum);
      SmartDashboard.putNumber("lockedInNum", lockedFiducialID);
      if (bestTarget != null) {
        for (PhotonTrackedTarget tar : bestTarget) {
          if (lockedFiducialID == -1) {
            System.out.println("WE LOCKED ON fiducial " + tar.getFiducialId());
            lockedFiducialID = tar.getFiducialId();
          }

          if (tar.getFiducialId() == lockedFiducialID) {
            Transform3d currentPose = tar.getBestCameraToTarget();

            xcontroller.setSetpoint(xSetpoint);
            ycontroller.setSetpoint(ySetpoint);
            s_Swerve.setAutoTurnHeading(VisionConstants.aprilTagAngle[tar.getFiducialId() - 1]);
            double xout = xcontroller.calculate(currentPose.getX());
            double yout = ycontroller.calculate(currentPose.getY());
            double thetaout = s_Swerve.getTurnPidSpeed();
            SmartDashboard.putNumber("x pid out", xout);
            SmartDashboard.putNumber("y pid out", yout);
            SmartDashboard.putNumber("theta pid out", thetaout);

            SmartDashboard.putNumber("x pid setpoint", xcontroller.getSetpoint());
            SmartDashboard.putNumber("y pid setpoint", ycontroller.getSetpoint());
            s_Swerve.drive(new Translation2d(-xout, -yout), -thetaout, false, false);
            SmartDashboard.putNumber("camera to pose x", currentPose.getX());
            SmartDashboard.putNumber("camera to pose y", currentPose.getY());
            SmartDashboard.putNumber("camera to pose z", currentPose.getZ());

            SmartDashboard.putNumber("id", tar.fiducialId);
            SmartDashboard.putNumber("pitch", tar.pitch);
            SmartDashboard.putNumber("yaw", tar.yaw);
            SmartDashboard.putNumber("ambiguity", tar.poseAmbiguity);
            SmartDashboard.putNumber("skew", tar.skew);

            SmartDashboard.putBoolean("xPID at setpt", xcontroller.atSetpoint());
            SmartDashboard.putBoolean("yPID at setpt", ycontroller.atSetpoint());
          } else {
            s_Swerve.stopDriving();
            System.err.println("LOST LOCKED ID, ENDING");
            isFinished = true;

          }
          // s_Swerve.setAutoTurnHeading(VisionConstants.aprilTagAngle[bestTarget.fiducialId - 1]);
        }
        autoEndcount ++;
        if(autoEndcount > 110) //greater than the number of cycle
        {
          System.out.println("haven't see a tag for too long... ending");
          isFinished = true;
        }
      }
    } else {
      System.err.println(
          "Something's wrong with photon,paste this line and search it globally to find it and look"
              + " at possible errors in the comment");
      /*Potential problem
       * 1. Coprocessor not powered
       * 2. Camera not connected
       * 3. Photonvision not seen on networktable
       */
      isFinished = true;

    }
    if(xKP.hasChanged())
    {
      xcontroller.setP(xKP.get());
    }
    if(xKI.hasChanged())
    {
      xcontroller.setI(xKI.get());
    }
    if(xKD.hasChanged())
    {
      xcontroller.setD(xKD.get());
    }

    if(yKP.hasChanged())
    {
      ycontroller.setP(yKP.get());
    }
    if(yKI.hasChanged())
    {
      ycontroller.setI(yKI.get());
    }
    if(yKD.hasChanged())
    {
      ycontroller.setD(yKD.get());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    s_Swerve.stopDriving();
    lockedFiducialID = -1;
    SmartDashboard.putBoolean("xPID at setpt", false);
    SmartDashboard.putBoolean("yPID at setpt", false);
    System.out.println("PHOTON ENDED");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(s_Photon.getResults().containsKey(cameraNum)
    && s_Photon.getResults().get(cameraNum) != null
    && s_Photon
        .getResults()
        .get(cameraNum)
        .isEmpty()){
          System.out.println("No cameras or camera arraylists are null");
          return true;
        }
      else if (xcontroller.atSetpoint() && ycontroller.atSetpoint())
      {
        System.out.println("At setpoint");
        return true;
      }
    else if (isFinished)
    {
      System.out.println("Is finished is set to true");
      return true;
    }
    else
    {
      return false;
    }
  }
}
