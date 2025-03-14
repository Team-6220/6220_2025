// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
public class CoralStationCmd extends Command
{
   // private ElevatorSubsystem elevator;
  private V2_SparkMaxWristSubsystem wrist;
  private ElevatorSubsystem elevatorSubsystem;
  private XboxController m_Controller;  

  private BooleanSupplier autoDrive;

  private boolean fieldRelative = true;
  private PhotonVisionSubsystem s_Photon;
  private Swerve s_Swerve;

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


  public CoralStationCmd(int cameraNum, Swerve s_Swerve, BooleanSupplier autoDrive)
  {
    // elevator = ElevatorSubsystem.getInstance();
    // addRequirements(elevator);
    wrist = V2_SparkMaxWristSubsystem.getInstance();
    elevatorSubsystem = ElevatorSubsystem.getInstance();
    s_Photon = PhotonVisionSubsystem.getInstance();
    this.m_Controller = null;
    this.cameraNum = cameraNum;
    this.s_Swerve = s_Swerve;
    this.autoDrive = autoDrive;
    addRequirements(wrist, elevatorSubsystem);
    addRequirements(s_Photon);
  }
  public CoralStationCmd(XboxController m_Controller, int cameraNum, Swerve s_Swerve)
  {
    // elevator = ElevatorSubsystem.getInstance();
    // addRequirements(elevator);
    wrist = V2_SparkMaxWristSubsystem.getInstance();
    elevatorSubsystem = ElevatorSubsystem.getInstance();
    s_Photon = PhotonVisionSubsystem.getInstance();
    this.m_Controller = m_Controller;
    this.cameraNum = cameraNum;
    this.s_Swerve = s_Swerve;
    addRequirements(wrist, elevatorSubsystem);
    addRequirements(s_Photon);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    wrist.setGoal(WristConstants.coralStation);
    elevatorSubsystem.setGoal(ElevatorConstants.E_CoralStation);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()  {
    // elevator.driveToGoal(ElevatorConstants.L2HeightRaw);
    double[] driverInputs = OIConstants.getDriverInputs(m_Controller);
    double xOutput = 0, yOutput = 0, rotationVal = 0;
    xOutput = driverInputs[0];
    yOutput = driverInputs[1];
    rotationVal = driverInputs[2];
    
    fieldRelative = true;

   if(autoDrive.getAsBoolean() || (m_Controller.getLeftBumperButton() || m_Controller.getRightBumperButton()))
   {
      if(!s_Photon.getResults().get(cameraNum).isEmpty()) 
      {
        PhotonTrackedTarget bestTarget = s_Photon.getBestTargets().get(cameraNum);
        if(bestTarget != null)
        {
          Transform3d currentPose = bestTarget.getBestCameraToTarget();
          
          xSetpoint = VisionConstants.centerCoralStationVisionX;
          ySetpoint = VisionConstants.centerCoralStationVisionY;

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
    wrist.driveToGoal();
    elevatorSubsystem.driveToGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("we ended");
    // elevator.stop();
    wrist.stop();
    elevatorSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
