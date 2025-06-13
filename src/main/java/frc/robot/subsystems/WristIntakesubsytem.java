// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.WristConstants;
import frc.robot.WristIntakeConstants;

import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.FovParamsConfigs;
import com.ctre.phoenix6.configs.ProximityParamsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.UpdateModeValue;

public class WristIntakesubsytem extends SubsystemBase {
  /** Creates a new WristIntake. */
  private static WristIntakesubsytem INSTANCE = null;

  private final TalonFX intakeMotor;
  private boolean coralInWrist;
  private boolean coralAtBack;
  private boolean hasExited;
  private boolean occupied;
  private double currentLimitToHold = -20;

  private CANrange canRange = new CANrange(WristConstants.CANRangeID);
  private CANrangeConfiguration canRangeConfigs = new CANrangeConfiguration();
  private FovParamsConfigs fovParamsConfigs = new FovParamsConfigs();
  private ProximityParamsConfigs proximityParamsConfigs = new ProximityParamsConfigs();
  private ToFParamsConfigs tofParamConfigs = new ToFParamsConfigs();

  private String tableKey = "WristIntake_";
  public TalonFXConfiguration wristIntakeConfig = new TalonFXConfiguration();

  public WristIntakesubsytem() {
    wristIntakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristIntakeConfig.MotorOutput.NeutralMode =
        WristIntakeConstants.INTAKENEU_NEUTRAL_MODE;

    wristIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable =
        WristIntakeConstants.enableCurrentLimit;
    wristIntakeConfig.CurrentLimits.SupplyCurrentLimit = WristIntakeConstants.maxCurrent;
    wristIntakeConfig.CurrentLimits.SupplyCurrentLowerLimit =
        WristIntakeConstants.currentLimit;
    wristIntakeConfig.CurrentLimits.SupplyCurrentLowerTime =
        WristIntakeConstants.maxCurrentTime;
    intakeMotor = new TalonFX(WristIntakeConstants.wristintakeMotorID);
    intakeMotor.getConfigurator().apply(wristIntakeConfig);
    
    fovParamsConfigs.FOVCenterX = 0;
    fovParamsConfigs.FOVRangeX = 27;
    fovParamsConfigs.FOVCenterY = 0;
    fovParamsConfigs.FOVRangeY = 10;
    canRangeConfigs.FovParams = fovParamsConfigs;
    
    canRangeConfigs.FutureProofConfigs = true;
    
    proximityParamsConfigs.MinSignalStrengthForValidMeasurement = 2500;
    proximityParamsConfigs.ProximityHysteresis = .05;
    proximityParamsConfigs.ProximityThreshold = .4;
    canRangeConfigs.ProximityParams = proximityParamsConfigs;
    
    tofParamConfigs.UpdateFrequency = 25;
    tofParamConfigs.UpdateMode = UpdateModeValue.ShortRangeUserFreq;
    canRangeConfigs.ToFParams = tofParamConfigs;

    canRange.getConfigurator().apply(canRangeConfigs);
  }

  public void simpleDrive(boolean reversed, double speed) {
    speed = reversed ? speed * -1 : speed;
    intakeMotor.set(speed);
  }

  public void intakeCoral() {
    occupied = true;
    simpleDrive(false, WristIntakeConstants.intakeSpeed);
  }

  public void ejectCoral() {
    occupied = true;
    simpleDrive(true, WristIntakeConstants.ejectSpeed);
  }

  public boolean canRangeTriggered(){
    return canRange.getIsDetected().getValue(); //depends on the proximity configs
  }

  public void endOccupied() {
    occupied = false;
  }

  @Override
  public void periodic() {
    if (!occupied && intakeMotor.getTorqueCurrent().getValueAsDouble() > currentLimitToHold) {
      // intakeMotor.set(-0.04);
      intakeMotor.setVoltage(-0.5);
    }
    if (intakeMotor.getTorqueCurrent().getValueAsDouble() <= currentLimitToHold) {
      intakeMotor.setVoltage(-0.15);
    }
    SmartDashboard.putBoolean("isOccupied", occupied);
    SmartDashboard.putNumber(
        tableKey + "stator current", intakeMotor.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        tableKey + "supply current", intakeMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(
        tableKey + "torque current", intakeMotor.getTorqueCurrent().getValueAsDouble());
    // This method will be called once per scheduler run
  }

  public static synchronized WristIntakesubsytem getInstance() {
    if (INSTANCE == null) {
      INSTANCE = new WristIntakesubsytem();
    }
    return INSTANCE;
  }
}
