// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.WristIntakeConstants;

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
  public TalonFXConfiguration wristIntakeConfig = new TalonFXConfiguration();


  public WristIntakesubsytem() {
        wristIntakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    wristIntakeConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.driveNeutralMode;

    wristIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.WristIntakeConstants.enableCurrentLimit;
    wristIntakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.WristIntakeConstants.maxCurrent;
    wristIntakeConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.WristIntakeConstants.currentLimit;
    wristIntakeConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.WristIntakeConstants.maxCurrentTime;
    intakeMotor = new TalonFX(WristIntakeConstants.wristintakeMotorID);
    intakeMotor.getConfigurator().apply(wristIntakeConfig);
  }
  public void simpleDrive(boolean reversed, double speed){
    speed = reversed ? speed * -1 : speed;
    intakeMotor.set(speed);
}
  
  public void intakeCoral(){
    simpleDrive(true, WristIntakeConstants.intakeSpeed);
  }
  public void ejectCoral(){
    simpleDrive(false, WristIntakeConstants.ejectSpeed);

  }
  @Override
  public void periodic() {

    // This method will be called once per scheduler run
  }

  public static synchronized WristIntakesubsytem getInstance()
  {
    if(INSTANCE == null)
    {
      INSTANCE = new WristIntakesubsytem();
    }
    return INSTANCE;
  }
}
