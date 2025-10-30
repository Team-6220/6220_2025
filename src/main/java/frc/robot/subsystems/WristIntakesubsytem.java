// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.WristIntakeConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

public class WristIntakesubsytem extends SubsystemBase {
  /** Creates a new WristIntake. */
  private static WristIntakesubsytem INSTANCE = null;

  private final TalonFX intakeMotor;
  private boolean coralInWrist;
  private boolean coralAtBack;
  private boolean hasExited;
  private boolean occupied;
  private double currentLimitToHold = -20;

  private String tableKey = "WristIntake_";
  public TalonFXConfiguration wristIntakeConfig = new TalonFXConfiguration();

  public WristIntakesubsytem() {
    wristIntakeConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    wristIntakeConfig.MotorOutput.NeutralMode = WristIntakeConstants.INTAKENEU_NEUTRAL_MODE;

    wristIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable =
        WristIntakeConstants.enableCurrentLimit;
    wristIntakeConfig.CurrentLimits.SupplyCurrentLimit = WristIntakeConstants.maxCurrent;
    wristIntakeConfig.CurrentLimits.SupplyCurrentLowerLimit = WristIntakeConstants.currentLimit;
    wristIntakeConfig.CurrentLimits.SupplyCurrentLowerTime = WristIntakeConstants.maxCurrentTime;
    intakeMotor = new TalonFX(WristIntakeConstants.wristintakeMotorID);
    intakeMotor.getConfigurator().apply(wristIntakeConfig);
  }

  public void simpleDrive(boolean reversed, double speed) {
    speed = reversed ? speed * -1 : speed;
    intakeMotor.set(speed);
  }

  public void simpleVoltageDrive(boolean reversed, double voltage){
    voltage = reversed? voltage * -1: voltage;
    intakeMotor.setVoltage(voltage);
  }

  public void intakeCoral() {
    occupied = true;
    simpleVoltageDrive(false, WristIntakeConstants.intakeVoltage);
  }

  public void ejectCoral() {
    occupied = true;
    simpleVoltageDrive(true, WristIntakeConstants.ejectVoltage);
  }

  public void endOccupied() {
    occupied = false;
  }

  @Override
  public void periodic() {
    // if (!occupied && intakeMotor.getTorqueCurrent().getValueAsDouble() > currentLimitToHold) {
    //   // intakeMotor.set(-0.04);
    //   intakeMotor.setVoltage(-0.5);
    // }
    // if (intakeMotor.getTorqueCurrent().getValueAsDouble() <= currentLimitToHold) {
    //   intakeMotor.setVoltage(-0.15);
    // }
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
