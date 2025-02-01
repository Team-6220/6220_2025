// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.FrontIntakeConstants;

public class frontIntakeSubsystem extends SubsystemBase {
  private static ElevatorSubsystem INSTANCE = null;

  private final TunableNumber FrontIntakeKp = new TunableNumber("FrontIntake kP", FrontIntakeConstants.frontIntakeKp);//TODO: match/make to constant.java
  private final TunableNumber FrontIntakeKi = new TunableNumber("FrontIntake kI", FrontIntakeConstants.frontIntakeKi);
  private final TunableNumber FrontIntakeKd = new TunableNumber("FrontIntake kD", FrontIntakeConstants.frontIntakeKd);
  private final TunableNumber FrontIntakeKg = new TunableNumber("FrontIntake kG",FrontIntakeConstants.frontIntakeKg);
  private final TunableNumber FrontIntakeKv = new TunableNumber("FrontIntake kV", FrontIntakeConstants.frontIntakeKv);
  private final TunableNumber FrontIntakeKs = new TunableNumber("FrontIntake kS", FrontIntakeConstants.frontIntakeKs);
  private final TunableNumber FrontIntakeIZone = new TunableNumber("FrontIntake izone", FrontIntakeConstants.frontIntakeIZone);//default 3
  private final TunableNumber FrontIntakeTolerance = new TunableNumber("FrontIntake tolerance", FrontIntakeConstants.frontIntakeTolerance);//default 1.5

  private final TunableNumber FrontIntakeMaxVel = new TunableNumber("FrontIntake max vel", FrontIntakeConstants.frontIntakeMaxVel);
  private final TunableNumber FrontIntakeMaxAccel = new TunableNumber("FrontIntake max accel", FrontIntakeConstants.frontIntakeMaxAccel);
  
  private final TunableNumber FrontIntakeSetpoint = new TunableNumber("FrontIntake goal setpoint", FrontIntakeConstants.frontIntakeEncoderOffset );
  
  private final SparkMax elevatorMotorLeft, elevatorMotorRight;
  private SparkMaxConfig motorLeftConfig, motorRightConfig;

  private final DutyCycleEncoder elevatorEncoder;
  private final ProfiledPIDController m_Controller;
  private ElevatorFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;
  private double lastUpdate = 0;
  public frontIntakeSubsystem() {
    elevatorMotorLeft = new SparkMax(FrontIntakeConstants.leftMotorID, MotorType.kBrushless);//TODO: CHANGE TO CONSTANTS
    elevatorMotorRight = new SparkMax(FrontIntakeConstants.rightMotorID, MotorType.kBrushless);

    motorLeftConfig
      .inverted(FrontIntakeConstants.leftMotorInvert)
      .idleMode(FrontIntakeConstants.leftMotorIdleMode);
    
    motorRightConfig
      .inverted(FrontIntakeConstants.rightMotorInvert)
      .idleMode(FrontIntakeConstants.rightMotorIdleMode)
      .follow(elevatorMotorLeft);
    
    elevatorMotorLeft.configure(motorLeftConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    m_Constraints = new TrapezoidProfile.Constraints(elevatorMaxVel.get(), elevatorMaxAccel.get());

    m_Controller = new ProfiledPIDController(
      elevatorKp.get(), 
      elevatorKi.get(), 
      elevatorKd.get(),
      m_Constraints);
    
    m_Feedforward = new ElevatorFeedforward(elevatorKs.get(), elevatorKg.get(), elevatorKv.get());

    m_Controller.setIZone(elevatorIZone.get());//not sure if we need this

    m_Controller.setTolerance(elevatorTolerance.get());//default 1.5

    elevatorEncoder = new DutyCycleEncoder(FrontIntakeConstants.elevatorEncoderID);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
