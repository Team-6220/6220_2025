// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SoftLimitConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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
  private final TunableNumber elevatorKs = new TunableNumber("Elevator kS", ElevatorConstants.elevatorKs);
  private final TunableNumber elevatorIZone = new TunableNumber("Elevator izone", ElevatorConstants.elevatorIZone);//default 3
  private final TunableNumber elevatorTolerance = new TunableNumber("Elevator tolerance", ElevatorConstants.elevatorTolerance);//default 1.5

  private final TunableNumber elevatorMaxVel = new TunableNumber("Elevator max vel", ElevatorConstants.elevatorMaxVel);
  private final TunableNumber elevatorMaxAccel = new TunableNumber("Elevator max accel", ElevatorConstants.elevatorMaxAccel);
  
  private final TunableNumber elevatorSetpointTest = new TunableNumber("Elevator goal setpoint", 30 );
  
  private final SparkMax elevatorMotorLeft, elevatorMotorRight;
  private SparkMaxConfig motorLeftConfig, motorRightConfig;

  private final blablaencoder elevatorEncoder;//TODO: ask what the encoders are 
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



  public ElevatorSubsystem()
  {
    elevatorMotorLeft = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);//TODO: CHANGE TO CONSTANTS
    elevatorMotorRight = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);

    motorLeftConfig
      .inverted(ElevatorConstants.leftMotorInvert)
      .idleMode(ElevatorConstants.leftMotorIdleMode);
    
    motorRightConfig
      .inverted(ElevatorConstants.rightMotorInvert)
      .idleMode(ElevatorConstants.rightMotorIdleMode)
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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  
  public void driveToGoal(double goal)
  {
    if(Timer.getFPGATimestamp() - 0.2 > lastUpdate)
    {
      resetPID();
    }

    lastUpdate = Timer.getFPGATimestamp();

    m_Controller.setGoal(goal);

    PIDOutput = m_Controller.calculate(getElevatorPosition());

    feedForwardOutput = m_Feedforward.calculate((m_Controller.getSetpoint().position+90) * Math.PI/180, m_Controller.getSetpoint().velocity);
        double calculatedSpeed = PIDOutput + feedForwardOutput;

        
        SmartDashboard.putNumber("Elevator Goal", goal);
  }
  
  public void resetPID()
  {
    m_Controller.reset(getElevatorPosition());
  }

  public double getElevatorPosition()
  {
    double elevatorPosition = 0.0;
    return elevatorPosition;
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
