// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class ClimberSubsystem extends SubsystemBase {
  private SparkMax climbMotor;
  private SparkMaxConfig climbMotorConfig = new SparkMaxConfig();
  private static ClimberSubsystem INSTANCE = null;
  /** Creates a new ClimberSubsystem. */
  public ClimberSubsystem() {
    climbMotor = new SparkMax(ClimberConstants.climbMotorID, MotorType.kBrushless);
    climbMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(30);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void simpleDrive(double motorOutput) {
    motorOutput *= 6;
    SmartDashboard.putNumber("Climber Output", motorOutput);
    climbMotor.setVoltage(motorOutput);
  }

  public void stop() {
    climbMotor.setVoltage(0);
  }
  
  public static synchronized ClimberSubsystem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new ClimberSubsystem();
    }
    return INSTANCE;
  }
}
