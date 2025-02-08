// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.RPM;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.MAXMotionConfig.MAXMotionPositionMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.WristConstants;

public class SparkMaxWrsitSubsystem extends SubsystemBase {
  String tableKey = "wrist_";

  SparkMax wristMotor = new SparkMax(WristConstants.WristMotorID, MotorType.kBrushless);
  SparkClosedLoopController wristPID = wristMotor.getClosedLoopController();

  AbsoluteEncoder wristAbsoluteEncoder = wristMotor.getAbsoluteEncoder();

  SparkMaxConfig wristConfig = new SparkMaxConfig();
  public SparkMaxWrsitSubsystem()
  {
    wristConfig.closedLoop.feedbackSensor(FeedbackSensor.kAbsoluteEncoder);
    wristConfig.closedLoop
    .pidf(WristConstants.kP, WristConstants.kI, WristConstants.kD, WristConstants.kFF)
    .maxOutput(.95)
    .maxMotion
      .allowedClosedLoopError(WristConstants.allowedClosedLoopError)
      .maxAcceleration(WristConstants.maxAcceleration)//Maximum Velocity is in units of Revolutions per Minute (RPM)
      .maxVelocity(WristConstants.maxVelocity)// Maximum Acceleration is in units of RPM per Second (RPM/s)
      .positionMode(MAXMotionPositionMode.kMAXMotionTrapezoidal);
    wristMotor.configure(wristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setPosition(double position)
  {
    if(position > WristConstants.wristMaxDegrees)
    {
      position = WristConstants.wristMaxDegrees;
    }
    if(position < WristConstants.wristMinDegrees)
    {
      position = WristConstants.wristMinDegrees;
    }

    wristPID.setReference(position, ControlType.kPosition);
  }

  public double getAbsolutePosition()
  {
    return wristAbsoluteEncoder.getPosition();
  }
  public static synchronized SparkMaxWrsitSubsystem getInstance() {
    return  new SparkMaxWrsitSubsystem();
}
}
