// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.TunableNumber;

public class ElevatorSubsystem extends SubsystemBase {
  private static ElevatorSubsystem INSTANCE = null;

  private final TunableNumber elevatorKp = new TunableNumber("Elevator kP", ElevatorConstants.elevatorKp);//TODO: match/make to constant.java
  private final TunableNumber elevatorKi = new TunableNumber("Elevator kI", ElevatorConstants.elevatorKi);
  private final TunableNumber elevatorKd = new TunableNumber("Elevator kD", ElevatorConstants.elevatorKd);
  private final TunableNumber elevatorKg = new TunableNumber("Elevator kG",ElevatorConstants.elevatorKg);
  private final TunableNumber elevatorKv = new TunableNumber("Elevator kV", ElevatorConstants.elevatorKv);
  private final TunableNumber elevatorKs = new TunableNumber("Elevator kS", ElevatorConstants.elevatorKs);

  private final TunableNumber elevatorMaxVel = new TunableNumber("Elevator max vel", ElevatorConstants.elevatorMaxVel);
  private final TunableNumber elevatorMaxAccel = new TunableNumber("Elevator max accel", ElevatorConstants.elevatorMaxAccel);
  
  private final TunableNumber elevatorSetpointTest = new TunableNumber("Elevator goal setpoint", 30 );
  
  private final SparkMax elevatorMotorLeft, elevatorMotorRight;
  private final blablaencoder elevatorEncoder;//TODO: ask what the encoders are 
  /** Creates a new ElevatorSubsystem. */
  public ElevatorSubsystem()
  {
    elevatorMotorLeft = new SparkMax(ElevatorConstants.leftMotorID, MotorType.kBrushless);//TODO: CHANGE TO CONSTANTS
    elevatorMotorRight = new SparkMax(ElevatorConstants.rightMotorID, MotorType.kBrushless);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
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
