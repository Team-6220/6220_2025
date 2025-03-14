// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.V2_SparkMaxWristSubsystem;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.Constants.WristConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Stage4CMD extends Command
{
  private ElevatorSubsystem elevator;
  private V2_SparkMaxWristSubsystem wrist;
  private XboxController m_Controller;

  private Swerve s_Swerve;

  private final BooleanSupplier leftControl, rightControl;

  private boolean fieldRelative = true;

  private TunableNumber elevHeight = new TunableNumber("l4 elev height", ElevatorConstants.E_L4);
  private TunableNumber wristDegrees = new TunableNumber("l4 wrist", WristConstants.L4);

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

  public Stage4CMD(Swerve s_Swerve, XboxController m_Controller, BooleanSupplier leftControl, BooleanSupplier rightControl, int cameraNum)
  {
    elevator = ElevatorSubsystem.getInstance();
    wrist = V2_SparkMaxWristSubsystem.getInstance();
    s_Photon = PhotonVisionSubsystem.getInstance();

    this.leftControl = leftControl;
    this.rightControl = rightControl;
    this.m_Controller = m_Controller;
    this.cameraNum = cameraNum;
    addRequirements(elevator);
    addRequirements(wrist);
    addRequirements(s_Swerve);
    addRequirements(s_Photon);
  }
  public Stage4CMD(Swerve s_Swerve, int cameraNum, BooleanSupplier leftControl, BooleanSupplier rightControl)
  {
    elevator = ElevatorSubsystem.getInstance();
    wrist = V2_SparkMaxWristSubsystem.getInstance();
    s_Photon = PhotonVisionSubsystem.getInstance();

    this.leftControl = () -> true;
    this.rightControl = rightControl;
    this.m_Controller = null;
    this.cameraNum = cameraNum;
    addRequirements(s_Photon);
    addRequirements(elevator);
    addRequirements(wrist);
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    elevator.setGoal(elevHeight.getDefault());
    wrist.setGoal(elevHeight.getDefault());
    VisionConstants.setTagXYHeightAngle();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()  {

    double[] driverInputs = OIConstants.getDriverInputs(m_Controller);
    double xOutput = 0, yOutput = 0, rotationVal = 0;
    xOutput = driverInputs[0];
    yOutput = driverInputs[1];
    rotationVal = driverInputs[2];
    
    fieldRelative = true;

    if(leftControl.getAsBoolean() || rightControl.getAsBoolean())
    {
      if(!s_Photon.getResults().get(cameraNum).isEmpty()) 
      {
        PhotonTrackedTarget bestTarget = s_Photon.getBestTargets().get(cameraNum);
        if(bestTarget != null)
        {
          Transform3d currentPose = bestTarget.getBestCameraToTarget();
          if(leftControl.getAsBoolean())
          {
            xSetpoint = VisionConstants.leftReefX;
            ySetpoint = VisionConstants.leftReefY;
          }
          if(rightControl.getAsBoolean())
          {
            xSetpoint = VisionConstants.rightReefX;
            ySetpoint = VisionConstants.rightReefY;
          }
          xcontroller.setSetpoint(xSetpoint);      
          ycontroller.setSetpoint(ySetpoint);
          s_Swerve.setAutoTurnHeading(VisionConstants.aprilTagAngle[bestTarget.getFiducialId()-1]);
          double xout = xcontroller.calculate(currentPose.getX());
          double yout = ycontroller.calculate(currentPose.getY());
          double thetaout = s_Swerve.getTurnPidSpeed();
          SmartDashboard.putNumber("x pid out", xout);
          SmartDashboard.putNumber("y pid out", yout);
          SmartDashboard.putNumber("theta pid out", thetaout);
          xOutput = xout;
          yOutput = yout;
          rotationVal = thetaout;
        }
        else
        {
          System.err.println("APRIL TAG NOT DETECTED");
        }
      }
    }
    s_Swerve.drive(new Translation2d(-xOutput, -yOutput), -rotationVal, fieldRelative,  false);
     
    // elevator.driveToGoal(ElevatorConstants.L2HeightRaw);
    if(elevHeight.hasChanged())
    {
      elevator.setGoal(elevHeight.get());
    }
    if(wristDegrees.hasChanged())
    {
      wrist.setGoal(wristDegrees.get());
    }
    wrist.driveToGoal();
    elevator.driveToGoal();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("we ended");
    elevator.stop();
    wrist.stop();
    s_Swerve.stopDriving();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
