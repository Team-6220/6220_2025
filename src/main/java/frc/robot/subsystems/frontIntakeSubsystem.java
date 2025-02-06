// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  
  private final SparkMax pivotMotorLeft, pivotMotorRight, frontMotor;
  private SparkMaxConfig motorLeftConfig, motorRightConfig, MotorfrontConfig;

  private final DutyCycleEncoder elevatorEncoder;
  private final ProfiledPIDController m_Controller;
  private ArmFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;
  private double lastUpdate = 0;

  private String tableKey = "frontIntake_";

  public frontIntakeSubsystem() {
    pivotMotorLeft = new SparkMax(FrontIntakeConstants.leftMotorID, MotorType.kBrushless);//TODO: CHANGE TO CONSTANTS
    pivotMotorRight = new SparkMax(FrontIntakeConstants.rightMotorID, MotorType.kBrushless);
    frontMotor = new SparkMax(FrontIntakeConstants.frontMotorID, MotorType.kBrushless);

    motorLeftConfig
      .inverted(FrontIntakeConstants.leftMotorInvert)
      .idleMode(FrontIntakeConstants.leftMotorIdleMode);
    MotorfrontConfig
      .inverted(FrontIntakeConstants.leftMotorInvert)
      .idleMode(FrontIntakeConstants.leftMotorIdleMode);
    
    motorRightConfig
      .inverted(FrontIntakeConstants.rightMotorInvert)
      .idleMode(FrontIntakeConstants.rightMotorIdleMode)
      .follow(pivotMotorLeft);
    
    pivotMotorLeft.configure(motorLeftConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    frontMotor.configure(MotorfrontConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_Constraints = new TrapezoidProfile.Constraints(FrontIntakeMaxVel.get(), FrontIntakeMaxAccel.get());

    m_Controller = new ProfiledPIDController(
      FrontIntakeKp.get(), 
      FrontIntakeKi.get(), 
      FrontIntakeKd.get(),
      m_Constraints);
    
    m_Feedforward = new ArmFeedforward(FrontIntakeKs.get(), FrontIntakeKg.get(), FrontIntakeKv.get());

    m_Controller.setIZone(FrontIntakeIZone.get());//not sure if we need this

    m_Controller.setTolerance(FrontIntakeTolerance.get());//default 1.5

    elevatorEncoder = new DutyCycleEncoder(FrontIntakeConstants.frontIntakeEncoderID);
  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
    SmartDashboard.putNumber(tableKey + "rawPosition", getElevatorPositionRaw());
    SmartDashboard.putNumber(tableKey + "Position", getElevatorPosition());
    SmartDashboard.putBoolean(tableKey + "atGoal", elevatorAtGoal());

    if(FrontIntakeKp.hasChanged()
        || FrontIntakeKi.hasChanged()
        || FrontIntakeKd.hasChanged())
        {
            m_Controller.setPID(FrontIntakeKp.get(),FrontIntakeKi.get(),FrontIntakeKd.get());
        }

        if(FrontIntakeKs.hasChanged()
        || FrontIntakeKg.hasChanged()
        || FrontIntakeKv.hasChanged()) {
            m_Feedforward = new ArmFeedforward(FrontIntakeKs.get(), FrontIntakeKg.get(), FrontIntakeKv.get());
        }

        if(FrontIntakeMaxVel.hasChanged()
        || FrontIntakeMaxAccel.hasChanged()) {
            m_Constraints = new TrapezoidProfile.Constraints(FrontIntakeMaxVel.get(),FrontIntakeMaxAccel.get());
            m_Controller.setConstraints(m_Constraints);
        }
        
        if(FrontIntakeIZone.hasChanged())
        {
          m_Controller.setIZone(FrontIntakeIZone.get());
        }

        if(FrontIntakeTolerance.hasChanged())
        {
          m_Controller.setTolerance(FrontIntakeTolerance.get());
        }
  }
    public void swingToGoal(double goal)
  {
    if(Timer.getFPGATimestamp() - 0.2 > lastUpdate)
    {
      resetPID();
    }

    lastUpdate = Timer.getFPGATimestamp();

    m_Controller.setGoal(goal);

    PIDOutput = m_Controller.calculate(getElevatorPosition());

    feedForwardOutput = m_Feedforward.calculate(m_Controller.getSetpoint().position, m_Controller.getSetpoint().velocity);
    double calculatedSpeed = PIDOutput + feedForwardOutput;

        
    SmartDashboard.putNumber("intake pivot Goal", goal);

    pivotMotorLeft.setVoltage(calculatedSpeed);
    pivotMotorRight.setVoltage(calculatedSpeed);
  }
  
  public void resetPID()
  {
    m_Controller.reset(getElevatorPosition());
  }

  /**Raw encoder value subtracted by the offset at zero*/
  public double getElevatorPosition()
  {
    return elevatorEncoder.get() - FrontIntakeSetpoint.get();
  }

  public double getElevatorPositionRaw()
  {
    return elevatorEncoder.get();
  }

  public void simpleDrive(double motorOutput)
  {
    pivotMotorLeft.set(motorOutput);
    pivotMotorRight.set(motorOutput);
  }

  public boolean elevatorAtGoal()
  {
    return m_Controller.atGoal();
  }
  public void spinFront(boolean on){
    if(on){
      frontMotor.set(FrontIntakeConstants.wheelSpeed);
    }
    else{
      frontMotor.set(0);
    } 
  }
  /**
     * Accesses the static instance of the ArmSubsystem singleton
     * @return ArmSubsystem Singleton Instance
     */
    public static synchronized ElevatorSubsystem getInstance() {
      if (INSTANCE == null) {
          INSTANCE = new ElevatorSubsystem();
      }
      return INSTANCE;
  }
}
