// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Robot;

import com.ctre.phoenix6.hardware.TalonFX;

public class WristSubsystem extends SubsystemBase {
  private static WristSubsystem INSTANCE = null;

  private String tableKey = "Shooter_";

  private TalonFX wristMotor;

  private final TunableNumber kP = new TunableNumber(tableKey + "kP", WristConstants.kP);
  private final TunableNumber kI = new TunableNumber(tableKey + "kI", WristConstants.kI);
  private final TunableNumber kD = new TunableNumber(tableKey + "kD", WristConstants.kD);
  private final TunableNumber kS = new TunableNumber(tableKey + "FF Ks", WristConstants.kS);
  private final TunableNumber kV = new TunableNumber(tableKey + "FF Kv", WristConstants.kV);
  private final TunableNumber kA = new TunableNumber(tableKey + "FF Ka", WristConstants.kA);

  private final ProfiledPIDController m_Controller;
  
  public WristSubsystem()
  {
    wristMotor = new TalonFX(WristConstants.wristMotorID);
    wristMotor.getConfigurator().apply(Robot.ctreConfigs.wristConfig);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public static synchronized WristSubsystem getInstance()
  {
    if(INSTANCE == null)
    {
      INSTANCE = new WristSubsystem();
    }
    return INSTANCE;
  }
}
