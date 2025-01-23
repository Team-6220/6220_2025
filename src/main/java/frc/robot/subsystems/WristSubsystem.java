// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialPositionVoltage;
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

  private final TunableNumber rampPeriodVoltage = new TunableNumber(tableKey + "rampPeriodVoltage", WristConstants.wristVoltageClosedLoopRampPeriod);

  private final DutyCycleEncoder wristEncoder;
  
  public WristSubsystem()
  {
    //Wrist "perminante" setup begin
    wristConfig.MotorOutput.Inverted = WristConstants.wristMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
    wristConfig.MotorOutput.NeutralMode = WristConstants.WRISTMOTOR_NEUTRAL_MODE_VALUE;

    wristConfig.CurrentLimits.SupplyCurrentLimitEnable = WristConstants.wristEnableCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLowerLimit = WristConstants.wristCurrentLimit;
    wristConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.wristCurrentThreshold;
    wristConfig.CurrentLimits.SupplyCurrentLowerTime = WristConstants.wristCurrentThresholdTime;
    wristConfig.Feedback.SensorToMechanismRatio = WristConstants.wristSensorToMechanismRatio;

    wristConfig.Slot0.kP = kP.get();
    wristConfig.Slot0.kI = kI.get();
    wristConfig.Slot0.kD = kD.get();

    wristConfig.Slot0.kS = kS.get();
    wristConfig.Slot0.kV = kV.get();
    wristConfig.Slot0.kA = kA.get();

    wristConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampPeriodVoltage.get();

    //Wrist "perminante" setup end
    wristMotor = new TalonFX(WristConstants.wristMotorID);
    wristMotor.getConfigurator().apply(wristConfig);

    wristEncoder = new DutyCycleEncoder(WristConstants.wristEncoderID);
  }

  public void updateConfigs()
  {
    wristConfig.Slot0.kP = kP.get();
    wristConfig.Slot0.kI = kI.get();
    wristConfig.Slot0.kD = kD.get();
    wristConfig.Slot0.kS = kS.get();
    wristConfig.Slot0.kV = kV.get();
    wristConfig.Slot0.kA = kA.get();

    wristConfig.DifferentialConstants.PeakDifferentialVoltage = WristConstants.peakDifferentialVoltage;
    wristConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = WristConstants.peakDifferentialTorqueCurrent;

    wristConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = rampPeriodVoltage.get();
    if(
      kP.hasChanged()||
      kI.hasChanged()||
      kD.hasChanged()||
      kS.hasChanged()||
      kV.hasChanged()||
      kA.hasChanged())
      {
        wristMotor.getConfigurator().apply(wristConfig);
      }
  }

  @Override
  public void periodic() {
    updateConfigs();
    SmartDashboard.putNumber(tableKey + "raw positoin", getPositionRaw());
  }

  public void setToPosition(Angle targetPosition)
  {
    DifferentialPositionVoltage motorPositionRequest = new DifferentialPositionVoltage(targetPosition, wristMotor.getPosition().getValue());
    SmartDashboard.putString(tableKey + "voltage request", motorPositionRequest.toString());
    wristMotor.setControl(motorPositionRequest);
  }

  public double getPositionDegrees()
  {
    return wristMotor.getPosition().getValueAsDouble() - WristConstants.wristMotorPositionOffset;
  }

  public double getPositionRaw()
  {
    return wristMotor.getPosition().getValueAsDouble();
  }

  public boolean getSoftTopLimitReached()
  {
    return wristMotor.getPosition().getValueAsDouble() >= WristConstants.wristSoftTopLimit;
  }

  public boolean getSoftLowLimitReached()
  {
    return wristMotor.getPosition().getValueAsDouble() >= WristConstants.wristSoftLowLimit;
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
