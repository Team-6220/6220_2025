// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.V2_SparkMaxWristSubsystem;
import frc.robot.Constants.*;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DeAlgeL2 extends Command {
  /** Creates a new DeAlge. */
  private V2_SparkMaxWristSubsystem wrist;
  private ElevatorSubsystem elevator;

  public DeAlgeL2() {
    // Use addRequirements() here to declare subsystem dependencies.
    wrist = V2_SparkMaxWristSubsystem.getInstance();
    elevator = ElevatorSubsystem.getInstance();
    addRequirements(wrist, elevator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setGoal(ElevatorConstants.eleDeAlgeL2);
    wrist.setGoal(WristConstants.deAlgeL2);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.driveToGoal();
    elevator.driveToGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.stop();
    wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
