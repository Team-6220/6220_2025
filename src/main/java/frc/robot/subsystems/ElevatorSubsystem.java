// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
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
import frc.robot.Constants;
import frc.robot.Constants.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem INSTANCE = null;

  private final TunableNumber elevatorKp = new TunableNumber("Elevator kP", ElevatorConstants.elevatorKp);//TODO: match/make to constant.java
  private final TunableNumber elevatorKi = new TunableNumber("Elevator kI", ElevatorConstants.elevatorKi);
  private final TunableNumber elevatorKd = new TunableNumber("Elevator kD", ElevatorConstants.elevatorKd);
  private final TunableNumber elevatorKg = new TunableNumber("Elevator kG",ElevatorConstants.elevatorKg);
  private final TunableNumber elevatorKv = new TunableNumber("Elevator kV", ElevatorConstants.elevatorKv);
  private final TunableNumber elevatorKa = new TunableNumber("Elevator kA", ElevatorConstants.elevatorKa);
  private final TunableNumber elevatorKs = new TunableNumber("Elevator kS", ElevatorConstants.elevatorKs);
  private final TunableNumber elevatorIZone = new TunableNumber("Elevator izone", ElevatorConstants.elevatorIZone);//default 3
  private final TunableNumber elevatorTolerance = new TunableNumber("Elevator tolerance", ElevatorConstants.elevatorTolerance);//default 1.5

  private final TunableNumber elevatorMaxVel = new TunableNumber("Elevator max vel", ElevatorConstants.elevatorMaxVel);
  private final TunableNumber elevatorMaxAccel = new TunableNumber("Elevator max accel", ElevatorConstants.elevatorMaxAccel);
  
  private final TunableNumber elevatorSetpoint = new TunableNumber("Elevator goal setpoint", ElevatorConstants.elevatorEncoderOffset );
  
  private final SparkMax elevatorMotorLeft, elevatorMotorRight;
  private SparkMaxConfig motorLeftConfig = new SparkMaxConfig(), motorRightConfig = new SparkMaxConfig();

  private final RelativeEncoder elevatorEncoder;
  /*
    Elevator PID & FF stuff
    see:
    https://docs.wpilib.org/en/stable/docs/software/advanced-controls/introduction/tuning-elevator.html
    for more detail
  */
  private final ProfiledPIDController m_Controller;
  private ElevatorFeedforward m_Feedforward;
  private TrapezoidProfile.Constraints m_Constraints;
  private double feedForwardOutput, PIDOutput;
  private double lastUpdate = 0;

  private String tableKey = "Elevator_";

  public ElevatorSubsystem()
  {
    elevatorMotorLeft = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);//TODO: CHANGE TO CONSTANTS
    elevatorMotorRight = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);

    motorLeftConfig
      .inverted(ElevatorConstants.leftMotorInvert)
      .idleMode(ElevatorConstants.leftMotorIdleMode)
      .smartCurrentLimit(ElevatorConstants.stallLimit, ElevatorConstants.freeLimit)
      .secondaryCurrentLimit(ElevatorConstants.freeLimit);
      //.follow(elevatorMotorRight); //Mainly because we're using the right encoder and we want to keep things organized
    
    motorRightConfig
      .inverted(ElevatorConstants.rightMotorInvert)
      .idleMode(ElevatorConstants.rightMotorIdleMode)
      .smartCurrentLimit(ElevatorConstants.stallLimit, ElevatorConstants.freeLimit)
      .secondaryCurrentLimit(ElevatorConstants.freeLimit);
    elevatorMotorLeft.configure(motorLeftConfig,ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    elevatorMotorRight.configure(motorRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_Constraints = new TrapezoidProfile.Constraints(elevatorMaxVel.get(), elevatorMaxAccel.get());

    m_Controller = new ProfiledPIDController(
      elevatorKp.get(), 
      elevatorKi.get(), 
      elevatorKd.get(),
      m_Constraints);
    
    m_Feedforward = new ElevatorFeedforward(elevatorKs.get(), elevatorKg.get(), elevatorKv.get(), elevatorKa.get());

    m_Controller.setIZone(elevatorIZone.get());//not sure if we need this

    m_Controller.setTolerance(elevatorTolerance.get());//default 1.5

    elevatorEncoder = elevatorMotorLeft.getEncoder(); //used right side because it provide positive values
    elevatorEncoder.setPosition(0);
  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber(tableKey + "Position", getElevatorPositionRaw());
    SmartDashboard.putNumber(tableKey + "Position Meters", getElevatorPositionMeters());
    SmartDashboard.putBoolean(tableKey + "atGoal", elevatorAtGoal());
    SmartDashboard.putNumber(tableKey + "leftCurrent", elevatorMotorLeft.getOutputCurrent());
    SmartDashboard.putNumber(tableKey + "rightCurrent", elevatorMotorRight.getOutputCurrent());
    SmartDashboard.putNumber(tableKey + "output", elevatorMotorLeft.getBusVoltage());
    SmartDashboard.putNumber(tableKey + "leftTemp", elevatorMotorLeft.getMotorTemperature());
    SmartDashboard.putNumber(tableKey + "rightTemp", elevatorMotorRight.getMotorTemperature());
    // System.out.println(getElevatorPosition());
    if(elevatorKp.hasChanged()
        || elevatorKi.hasChanged()
        || elevatorKd.hasChanged())
        {
            m_Controller.setPID(elevatorKp.get(),elevatorKi.get(),elevatorKd.get());
        }

        if(elevatorKs.hasChanged()
        || elevatorKg.hasChanged()
        || elevatorKv.hasChanged()
        || elevatorKa.hasChanged()) {
            m_Feedforward = new ElevatorFeedforward(elevatorKs.get(), elevatorKg.get(), elevatorKv.get(),elevatorKa.get());
            System.out.println("ff changed w/ ks" + elevatorKs.get() + ",kg" + elevatorKg.get() + ",kv" + elevatorKv.get() +",ka"+ elevatorKa.get());
          }

        if(elevatorMaxVel.hasChanged()
        || elevatorMaxAccel.hasChanged()) {
            m_Constraints = new TrapezoidProfile.Constraints(elevatorMaxVel.get(), elevatorMaxAccel.get());
            m_Controller.setConstraints(m_Constraints);
            System.out.println("constraints created w/ new max vel" + elevatorMaxVel.get() + ", max accel" + elevatorMaxAccel.get());
        }
        
        if(elevatorIZone.hasChanged())
        {
          m_Controller.setIZone(elevatorIZone.get());
        }

        if(elevatorTolerance.hasChanged())
        {
          m_Controller.setTolerance(elevatorTolerance.get());
        }
  }
  
  public void driveToGoal(double goal)
  {
    if(Timer.getFPGATimestamp() - 0.2 > lastUpdate)
    {
      resetPID();
    }

    lastUpdate = Timer.getFPGATimestamp();

    if(goal > ElevatorConstants.upperEncoderExtreme)
    {
      goal = ElevatorConstants.upperEncoderExtreme;
    }

    if(goal < ElevatorConstants.lowerEncoderExtreme)
    {
      goal = ElevatorConstants.lowerEncoderExtreme;
    }

    m_Controller.setGoal(goal);

    PIDOutput = m_Controller.calculate(getElevatorPositionMeters());

    feedForwardOutput = m_Feedforward.calculate(m_Controller.getSetpoint().position, m_Controller.getSetpoint().velocity);
    double calculatedSpeed = PIDOutput + feedForwardOutput;

        
    SmartDashboard.putNumber(tableKey + "Elevator Goal", goal);

    elevatorMotorLeft.setVoltage(calculatedSpeed);
    elevatorMotorRight.setVoltage(calculatedSpeed);
    SmartDashboard.putNumber(tableKey + "positionError", m_Controller.getPositionError());
    // SmartDashboard.putNumber(tableKey ;
    SmartDashboard.putNumber(tableKey + "calculated Speed", calculatedSpeed);
    SmartDashboard.putNumber(tableKey + "ffOutput", feedForwardOutput);
    SmartDashboard.putNumber(tableKey + "PIDOutput", PIDOutput);
    // SmartDashboard.putNumber(tableKey + "current Elevator pos", getElevatorPositionMeters());
  }

  // public double getPositionMeters()
  // {
  //   return elevatorEncoder.getPosition() / (1/20) * SproketRadius * (2*Math.PI);
  // }
  
  public void resetPID()
  {
    m_Controller.reset(getElevatorPositionMeters());
  }

  /**In meters*/
  public double getElevatorPositionMeters()
  {
    //Pivit position
    double elevatorPosition = elevatorEncoder.getPosition() * (1/20) * 5.4978 * .0254 * 2;//* gear reatio * circum of sprocket * convert inches to meters * second stage move x2 as fast as first stage*/
    return elevatorPosition;
  }

  public double getElevatorPositionRaw()
  {
    return elevatorEncoder.getPosition();
  }


  public void simpleDrive(double motorOutput)
  {
    motorOutput *= 12 ;
    SmartDashboard.putNumber(tableKey + "output", motorOutput);
    SmartDashboard.putNumber(tableKey + "sd_velocity", elevatorEncoder.getVelocity());
    elevatorMotorLeft.setVoltage(motorOutput);
    elevatorMotorRight.setVoltage(motorOutput);
  }

  public boolean elevatorAtGoal()
  {
    return m_Controller.atGoal();
  }

  public void stop(){
    elevatorMotorRight.set(0);
    m_Controller.reset(getElevatorPositionRaw());
  }

  public void resetEncoder()
  {
    elevatorEncoder.setPosition(0);
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