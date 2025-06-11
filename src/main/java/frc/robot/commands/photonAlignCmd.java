// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose3d;
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
  private final TunableNumber xMaxVel = new TunableNumber("x MaxVel", SwerveConstants.xMaxVel);
  private final TunableNumber xMaxAccel = new TunableNumber("x Accel", SwerveConstants.xMaxAccel);

  private final TunableNumber yKP = new TunableNumber("y kP", SwerveConstants.yKP);
  private final TunableNumber yKI = new TunableNumber("y kI", SwerveConstants.yKI);
  private final TunableNumber yKD = new TunableNumber("y kD", SwerveConstants.yKD);
  private final TunableNumber yMaxVel = new TunableNumber("y MaxVel", SwerveConstants.yMaxVel);
  private final TunableNumber yMaxAccel = new TunableNumber("y Accel", SwerveConstants.yMaxAccel);
  private int cameraNum;
  private double robotXSetpoint, robotYSetpoint; //robot relative
  private int lockedFiducialID = -1;
  private PIDController xcontroller = new PIDController(xKP.get(), xKI.get(), xKD.get());
  private PIDController ycontroller = new PIDController(yKP.get(), yKI.get(), yKD.get());

  // private PhotonTrackedTarget bestTarget;

  /** Creates a new photonAlign. */
  public photonAlignCmd(int cameraNum, Swerve s_Swerve, double robotXSetpoint, double robotYSetpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    s_Photon = PhotonVisionSubsystem.getInstance(VisionConstants.cameraNames);
    this.s_Swerve = s_Swerve;
    addRequirements(s_Photon, s_Swerve);
    this.cameraNum = cameraNum;
    this.robotXSetpoint = robotXSetpoint;
    this.robotYSetpoint = robotYSetpoint;
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
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.print("Photon vision cmd running");
    s_Photon.updatePhoton();
    if (s_Photon.getResults().containsKey(cameraNum)
        && s_Photon.getResults().get(cameraNum) != null
        && !s_Photon.getResults().get(cameraNum).isEmpty()) {
      List<PhotonTrackedTarget> bestTarget = s_Photon.getBestTargets().get(cameraNum);
      SmartDashboard.putNumber("lockedInNum", lockedFiducialID);
      if (bestTarget != null) {
        for (PhotonTrackedTarget tar : bestTarget) {
          if (lockedFiducialID == -1) {
            lockedFiducialID = tar.getFiducialId();
          }

          if (tar.getFiducialId() == lockedFiducialID) {
            Transform3d cameraToTag = tar.getBestCameraToTarget();
            
            Transform3d robotToTag = VisionConstants.robotCenterToCamera[cameraNum].plus(cameraToTag);

            xcontroller.setPID(xKP.get(), xKI.get(), xKD.get());
            ycontroller.setPID(yKP.get(),yKI.get(), yKD.get());

            double xout_robot = xcontroller.calculate(robotToTag.getX(), robotXSetpoint);
            double yout_robot = ycontroller.calculate(robotToTag.getY(), robotYSetpoint);
            
            s_Swerve.setAutoTurnHeading(VisionConstants.aprilTagYaw[tar.getFiducialId() - 1]);
            
            double thetaout = s_Swerve.getTurnPidSpeed();
            SmartDashboard.putNumber("x pid out", xout_robot);
            SmartDashboard.putNumber("y pid out", yout_robot);
            SmartDashboard.putNumber("theta pid out", thetaout);


            s_Swerve.drive(new Translation2d(-xout_robot, -yout_robot), thetaout, false, false);
            SmartDashboard.putNumber("camera to pose x", cameraToTag.getX());
            SmartDashboard.putNumber("camera to pose y", cameraToTag.getY());
            SmartDashboard.putNumber("camera to pose z", cameraToTag.getZ());

            SmartDashboard.putNumber("id", tar.fiducialId);
            SmartDashboard.putNumber("pitch", tar.pitch);
            SmartDashboard.putNumber("yaw", tar.yaw);
            SmartDashboard.putNumber("ambiguity", tar.poseAmbiguity);
            SmartDashboard.putNumber("skew", tar.skew);

            SmartDashboard.putBoolean("xPID at setpt", xcontroller.atSetpoint());
            SmartDashboard.putBoolean("yPID at setpt", ycontroller.atSetpoint());
            SmartDashboard.putNumber("Robot X setpoint", robotXSetpoint);
            SmartDashboard.putNumber("Robot Y setpiont", robotYSetpoint);
            SmartDashboard.putNumber("X PID Output", xout_robot);
            SmartDashboard.putNumber("Y PID Output", yout_robot);
          } else {
            s_Swerve.stopDriving();
          }

          // s_Swerve.setAutoTurnHeading(VisionConstants.aprilTagAngle[bestTarget.fiducialId - 1]);
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
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("PHOTON ENDED");
    s_Swerve.stopDriving();
    lockedFiducialID = -1;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return s_Photon.getResults().containsKey(cameraNum)
        && s_Photon.getResults().get(cameraNum) != null
        && s_Photon
            .getResults()
            .get(cameraNum)
            .isEmpty(); // if there's no tag automatically stop it from driving
  }
}
