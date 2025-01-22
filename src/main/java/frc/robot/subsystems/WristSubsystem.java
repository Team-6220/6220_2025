// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class WristSubsystem extends SubsystemBase {
  private static WristSubsystem INSTANCE = null;

  private String tableKey = "Shooter_";

  private TalonFX wristMotor;

  public TalonFXConfiguration wristConfig = new TalonFXConfiguration();


  private final TunableNumber kP = new TunableNumber(tableKey + "kP", WristConstants.kP);
  private final TunableNumber kI = new TunableNumber(tableKey + "kI", WristConstants.kI);
  private final TunableNumber kD = new TunableNumber(tableKey + "kD", WristConstants.kD);
  private final TunableNumber kS = new TunableNumber(tableKey + "FF Ks", WristConstants.kS);
  private final TunableNumber kV = new TunableNumber(tableKey + "FF Kv", WristConstants.kV);
  private final TunableNumber kA = new TunableNumber(tableKey + "FF Ka", WristConstants.kA);

  private final TunableNumber rampPeriod

  private final ProfiledPIDController m_Controller;
  
  public WristSubsystem()
  {
    //Wrist "perminante" setup begin
    wristConfig.MotorOutput.Inverted = WristConstants.wristMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    wristConfig.MotorOutput.NeutralMode = WristConstants.WRISTMOTOR_NEUTRAL_MODE_VALUE;

    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = WristConstants.wristEnableCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLowerLimit = WristConstants.wristCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.wristCurrentThreshold;
    wristConfig.CurrentLimits.SupplyCurrentLowerTime = WristConstants.wristCurrentThresholdTime;

    //Wrist "perminante" setup end
    wristMotor = new TalonFX(WristConstants.wristMotorID);
    wristMotor.getConfigurator().apply(Robot.ctreConfigs.wristConfig);
  }

  public void updateConfig()
  {
    if()
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
