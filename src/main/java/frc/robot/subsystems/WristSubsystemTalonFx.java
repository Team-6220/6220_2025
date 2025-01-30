// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//DO NOT USE -- INCOMPLETE
//USE THE PID IN PHOENIX TUNER INSTEAD OF TUNABLE NUMBERS


package frc.robot.subsystems;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.WristConstants;
import frc.robot.Robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialPositionVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;

public class WristSubsystemTalonFx extends SubsystemBase {
  private static WristSubsystemTalonFx INSTANCE = null;

  private String tableKey = "Shooter_";

  private TalonFX wristMotor;

  private double positionToHold = 0;

  public TalonFXConfiguration wristConfig = new TalonFXConfiguration();

  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);


  private final TunableNumber kP = new TunableNumber(tableKey + "kP", WristConstants.kP);
  private final TunableNumber kI = new TunableNumber(tableKey + "kI", WristConstants.kI);
  private final TunableNumber kD = new TunableNumber(tableKey + "kD", WristConstants.kD);
  private final TunableNumber kS = new TunableNumber(tableKey + "FF Ks", WristConstants.kS);
  private final TunableNumber kV = new TunableNumber(tableKey + "FF Kv", WristConstants.kV);
  private final TunableNumber kA = new TunableNumber(tableKey + "FF Ka", WristConstants.kA);

  private final TunableNumber rampPeriodVoltage = new TunableNumber(tableKey + "rampPeriodVoltage", WristConstants.wristVoltageClosedLoopRampPeriod);

  private final DutyCycleEncoder wristEncoder;
  
  public WristSubsystemTalonFx()
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

    wristConfig.DifferentialConstants.PeakDifferentialVoltage = WristConstants.peakDifferentialVoltage;
    wristConfig.DifferentialConstants.PeakDifferentialTorqueCurrent = WristConstants.peakDifferentialTorqueCurrent;

    wristConfig.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

    wristConfig.Voltage.withPeakForwardVoltage(Volts.of(8))
      .withPeakReverseVoltage(Volts.of(-8));

    //Wrist "perminante" setup end
    wristMotor = new TalonFX(WristConstants.wristMotorID);
    wristMotor.getConfigurator().apply(wristConfig);

    wristEncoder = new DutyCycleEncoder(WristConstants.wristEncoderID);
  }

  public void updateConfigs()
  {
    // SmartDashboard.putNumber(tableKey + "kp", wristConfig.Slot0.kP);
    // System.out.print("blablabla");
    if(
      kP.hasChanged()||
      kI.hasChanged()||
      kD.hasChanged()||
      kS.hasChanged()||
      kV.hasChanged()||
      kA.hasChanged()
      )
      {
        // TalonFXConfiguration tesTalonFXConfiguration = wristConfig;
        wristConfig.Slot0.kP = kP.get();
        wristConfig.Slot0.kI = kI.get();
        wristConfig.Slot0.kD = kD.get();
        wristConfig.Slot0.kS = kS.get();
        wristConfig.Slot0.kV = kV.get();
        wristConfig.Slot0.kA = kA.get();
        wristMotor.getConfigurator().apply(wristConfig);
        System.out.println("updated!\n\n\n\n\n\n\n\n\n\n\n\noh ye");
      }
      // System.out.print("updateConfigs");
  }

  @Override
  public void periodic() {
    updateConfigs();
    
    SmartDashboard.putNumber(tableKey + "raw positoin", getPositionRaw());
    SmartDashboard.putBoolean("is motor upper limit meet", getSoftTopLimitReached());
    SmartDashboard.putBoolean("is motor low limit meet", getSoftLowLimitReached());
  }

  public void setToPosition(double targetPosition)
  {
    targetPosition = targetPosition > WristConstants.wristSoftTopLimit ?//DO NOT DELETE: to make sure wrist doesn't snap
    WristConstants.wristSoftTopLimit : targetPosition < WristConstants.wristSoftLowLimit ?//DO NOT DELETE: to make sure wrist doesn't snap
    WristConstants.wristSoftLowLimit : targetPosition;//DO NOT DELETE: to make sure wrist doesn't snap
    wristMotor.setControl(m_positionVoltage.withPosition(targetPosition)
    .withLimitForwardMotion(getSoftTopLimitReached())
    .withLimitReverseMotion(getSoftLowLimitReached()));
    SmartDashboard.putString(tableKey + "voltage request", m_positionVoltage.toString());
  }

  public void updatePosition(double targetPosition)
  {
    positionToHold = targetPosition;
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
    return wristMotor.getPosition().getValueAsDouble() <= WristConstants.wristSoftLowLimit;
  } 

  public static synchronized WristSubsystemTalonFx getInstance()
  {
    if(INSTANCE == null)
    {
      INSTANCE = new WristSubsystemTalonFx();
    }
    return INSTANCE;
  }
}
