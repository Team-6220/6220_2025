// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.Constants.FrontIntakeConstants;
import frc.robot.Robot;

public class frontIntakeSubsystem extends SubsystemBase {
  private static frontIntakeSubsystem INSTANCE = null;

  private final TunableNumber FrontIntakeKp = new TunableNumber("FrontIntake kP", FrontIntakeConstants.frontIntakeKp);//TODO: match/make to constant.java
  private final TunableNumber FrontIntakeKi = new TunableNumber("FrontIntake kI", FrontIntakeConstants.frontIntakeKi);
  private final TunableNumber FrontIntakeKd = new TunableNumber("FrontIntake kD", FrontIntakeConstants.frontIntakeKd);
  private final TunableNumber FrontIntakeKg = new TunableNumber("FrontIntake kG",FrontIntakeConstants.frontIntakeKg);
  private final TunableNumber FrontIntakeKv = new TunableNumber("FrontIntake kV", FrontIntakeConstants.frontIntakeKv);
  private final TunableNumber FrontIntakeKs = new TunableNumber("FrontIntake kS", FrontIntakeConstants.frontIntakeKs);
  private final TunableNumber FrontIntakeKa = new TunableNumber("FrontIntake kA", FrontIntakeConstants.frontIntakeKa);
  private final TunableNumber FrontIntakeIZone = new TunableNumber("FrontIntake izone", FrontIntakeConstants.frontIntakeIZone);//default 3
  private final TunableNumber FrontIntakeTolerance = new TunableNumber("FrontIntake tolerance", FrontIntakeConstants.frontIntakeTolerance);//default 1.5

  private final TunableNumber FrontIntakeMaxVel = new TunableNumber("FrontIntake max vel", FrontIntakeConstants.frontIntakeMaxVel);
  private final TunableNumber FrontIntakeMaxAccel = new TunableNumber("FrontIntake max accel", FrontIntakeConstants.frontIntakeMaxAccel);

  private final TunableNumber frontIntakeIdleVoltage = new TunableNumber("lower intake idle voltage", FrontIntakeConstants.idleSpinVoltage);
  private final TunableNumber frontIntakeVoltage = new TunableNumber("lower intake voltage", FrontIntakeConstants.wheelSpeed);
  private double idleOutVolt = FrontIntakeConstants.idleSpinVoltage, intakeOutVolt = FrontIntakeConstants.wheelSpeed; //place holder when update the voltages
  
  private final SparkMax pivotMotorLeft, pivotMotorRight;
  private final TalonFX frontMotor;
  public TalonFXConfiguration lowerIntakeConfig = new TalonFXConfiguration();

  private SparkMaxConfig motorLeftConfig = new SparkMaxConfig(), motorRightConfig = new SparkMaxConfig();

  private final DutyCycleEncoder lowerintakeEncoder;
  private final ProfiledPIDController m_Controller;
  private ArmFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;
  private double lastUpdate = 0;

  private String tableKey = "frontIntake_";

  public frontIntakeSubsystem() {
    pivotMotorLeft = new SparkMax(FrontIntakeConstants.leftMotorID, MotorType.kBrushless);//TODO: CHANGE TO CONSTANTS
    pivotMotorRight = new SparkMax(FrontIntakeConstants.rightMotorID, MotorType.kBrushless);
    frontMotor = new TalonFX(FrontIntakeConstants.frontMotorID);

    lowerIntakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;// positive is intake
    lowerIntakeConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.driveNeutralMode;

    lowerIntakeConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.FrontIntakeConstants.enableCurrentLimit;
    lowerIntakeConfig.CurrentLimits.SupplyCurrentLimit = Constants.FrontIntakeConstants.maxCurrent;
    lowerIntakeConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.FrontIntakeConstants.currentLimit;
    lowerIntakeConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.FrontIntakeConstants.maxCurrentTime;
    lowerIntakeConfig.CurrentLimits.StatorCurrentLimitEnable = Constants.FrontIntakeConstants.enableStatorCurrentLimit;
    lowerIntakeConfig.CurrentLimits.StatorCurrentLimit = Constants.FrontIntakeConstants.maxStatorCurrent;
    

    frontMotor.getConfigurator().apply(lowerIntakeConfig);

    motorLeftConfig
      .inverted(FrontIntakeConstants.leftMotorInvert)
      .idleMode(FrontIntakeConstants.leftMotorIdleMode)
      .smartCurrentLimit(FrontIntakeConstants.stallLimit, FrontIntakeConstants.freeLimit);
    motorRightConfig
      .inverted(FrontIntakeConstants.rightMotorInvert)
      .idleMode(FrontIntakeConstants.rightMotorIdleMode)
      .smartCurrentLimit(FrontIntakeConstants.stallLimit, FrontIntakeConstants.freeLimit);
    
    pivotMotorLeft.configure(motorLeftConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    pivotMotorRight.configure(motorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_Constraints = new TrapezoidProfile.Constraints(FrontIntakeMaxVel.get(), FrontIntakeMaxAccel.get());

    m_Controller = new ProfiledPIDController(
      FrontIntakeKp.get(), 
      FrontIntakeKi.get(), 
      FrontIntakeKd.get(),
      m_Constraints);
    
    m_Feedforward = new ArmFeedforward(FrontIntakeKs.get(), FrontIntakeKg.get(), FrontIntakeKv.get(), FrontIntakeKa.get());

    m_Controller.setIZone(FrontIntakeIZone.get());//not sure if we need this

    m_Controller.setTolerance(FrontIntakeTolerance.get());//default 1.5

    lowerintakeEncoder = new DutyCycleEncoder(2);
  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run
    SmartDashboard.putNumber(tableKey + "Position", getPosition());
    SmartDashboard.putNumber(tableKey+"rawPosition", lowerintakeEncoder.get());
    SmartDashboard.putBoolean(tableKey + "atGoal", controllerAtGoal());
    SmartDashboard.putNumber(tableKey + "leftCurrent", pivotMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber(tableKey + "rightCurrent", pivotMotorRight.getOutputCurrent());
    SmartDashboard.putNumber(tableKey +"motorOutputLeft", pivotMotorLeft.getAppliedOutput());
    SmartDashboard.putNumber(tableKey +"motorOutputRight", pivotMotorRight.getAppliedOutput());
    SmartDashboard.putNumber(tableKey + "motorOutputManuel", 0);
    SmartDashboard.putNumber(tableKey + "intakeMotorTemp", frontMotor.getDeviceTemp().getValueAsDouble());
    SmartDashboard.putNumber(tableKey + "intakeMotorTorqueCurrentDraw", frontMotor.getTorqueCurrent().getValueAsDouble());
    SmartDashboard.putNumber(tableKey + "intakeMotorSupplyCurrentDraw", frontMotor.getSupplyCurrent().getValueAsDouble());
    SmartDashboard.putNumber(tableKey +"intake current limit", lowerIntakeConfig.CurrentLimits.SupplyCurrentLimit);
    SmartDashboard.putNumber(tableKey +"intakeMotorStatorCurrentLimit", frontMotor.getStatorCurrent().getValueAsDouble());
    // SmartDashboard.putNumber(tableKey + "encoder vel");

    if(FrontIntakeKp.hasChanged()
        || FrontIntakeKi.hasChanged()
        || FrontIntakeKd.hasChanged())
        {
            m_Controller.setPID(FrontIntakeKp.get(),FrontIntakeKi.get(),FrontIntakeKd.get());
            System.out.println("new PID;P:" + FrontIntakeKp.get() + "I:" + FrontIntakeKi.get() + "D:" + FrontIntakeKd.get());
        }

        if(FrontIntakeKs.hasChanged()
        || FrontIntakeKg.hasChanged()
        || FrontIntakeKv.hasChanged()) {
            m_Feedforward = new ArmFeedforward(FrontIntakeKs.get(), FrontIntakeKg.get(), FrontIntakeKv.get(), FrontIntakeConstants.frontIntakeKa);
            System.out.println("new ff;s:" + FrontIntakeKs.get() + "g:" + FrontIntakeKg.get() + "v:" + FrontIntakeKv.get());
          }

        if(FrontIntakeMaxVel.hasChanged()
        || FrontIntakeMaxAccel.hasChanged()) {
            m_Constraints = new TrapezoidProfile.Constraints(FrontIntakeMaxVel.get(),FrontIntakeMaxAccel.get());
            m_Controller.setConstraints(m_Constraints);
            System.out.println("new contraints;max vel:" + FrontIntakeMaxVel.get() + "max accel:" + FrontIntakeMaxAccel.get());
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

  public void setGoal(double goal)
  {
    resetPID();

    if(goal > FrontIntakeConstants.maxDegrees)
    {
      goal = FrontIntakeConstants.maxDegrees;
    }
    if(goal < FrontIntakeConstants.minDegrees)
    {
      goal = FrontIntakeConstants.minDegrees;
    }

    m_Controller.setGoal(goal);
  }

  public void swingToGoal()
  {
    // SmartDashboard.putNumber(tableKey + "Position", goal);
    feedForwardOutput = m_Feedforward.calculate((m_Controller.getSetpoint().position)*Math.PI/180, m_Controller.getSetpoint().velocity*Math.PI/180);
    
    lastUpdate = Timer.getFPGATimestamp();

    PIDOutput = m_Controller.calculate(getPosition());

    double calculatedOutput = PIDOutput + feedForwardOutput;

        
    SmartDashboard.putNumber(tableKey + "ffOut", feedForwardOutput);
    SmartDashboard.putNumber(tableKey + "pidOut", PIDOutput);
    SmartDashboard.putNumber(tableKey + "calculatedOutput", calculatedOutput);
    SmartDashboard.putNumber(tableKey + "setPoint", m_Controller.getSetpoint().position);
    SmartDashboard.putNumber(tableKey + "setPointVelocity", m_Controller.getSetpoint().velocity);
    SmartDashboard.putBoolean(tableKey + "atsetpoint", m_Controller.atSetpoint());
    SmartDashboard.putNumber(tableKey + "goal", m_Controller.getGoal().position);
    pivotMotorLeft.setVoltage(calculatedOutput);
    pivotMotorRight.setVoltage(calculatedOutput);
  }
  
  public void resetPID()
  {
    
    m_Controller.reset(getPosition());
  }

  /**Raw encoder value subtracted by the offset at zero*/
  public double getPosition()
  {
    return ((lowerintakeEncoder.get()) *(24.0/32.0) * 360.0)-304+185.5+31.5
    ;//(encoder value - offset) * gear ratio from shaft to encoder *360 to get degrees
  }
  
  public void simpleDrive(double motorOutput)
  {
    // motorOutput *= 12;
    SmartDashboard.putNumber(tableKey + "motorOutputManuel", motorOutput);
    pivotMotorLeft.set(motorOutput);
    pivotMotorRight.set(motorOutput);
  }

  public boolean controllerAtGoal()
  {
    return m_Controller.atGoal();
  }

  public void simpleintakeDrive(double speed){
    frontMotor.setVoltage(speed);
  }

  public void setFront(double volt){
    frontMotor.setVoltage(volt);
  }

  /**
   * @deprecated
   * USE setFront() INSTEAD (the intake for algae is the outtake for coarl and the intake for coral is the outtake for algae, too confusing)
   * @param spin
   * @param intake
   */
  public void spinFront(boolean spin, boolean intake){
    if(frontIntakeVoltage.hasChanged())
    {
      intakeOutVolt = frontIntakeVoltage.get();
    }
    if(spin&&intake){
      frontMotor.setVoltage(intakeOutVolt);
    }
    if(spin&&!intake){
      frontMotor.setVoltage(-intakeOutVolt);
    }
    if(!spin){
      frontMotor.set(0);
    } }
    public void maintainFront()
    {
      if(frontIntakeIdleVoltage.hasChanged())
      {
        idleOutVolt = frontIntakeIdleVoltage.get();
      }
      frontMotor.setVoltage(-idleOutVolt);
    }

    public void setMaxVel(double maxVel) {
      FrontIntakeMaxVel.setDefault(maxVel);
      SmartDashboard.putNumber("FrontIntake max vel", maxVel);
    }

    public void setMaxAccel(double maxAccel) {
      FrontIntakeMaxAccel.setDefault(maxAccel);
      SmartDashboard.putNumber("FrontIntake max accel", maxAccel);
    }
  /**
     * Accesses the static instance of the ArmSubsystem singleton
     * @return ArmSubsystem Singleton Instance
     */
    public static synchronized frontIntakeSubsystem getInstance() {
      if (INSTANCE == null) {
          INSTANCE = new frontIntakeSubsystem();
      }
      return INSTANCE;
  }
}
