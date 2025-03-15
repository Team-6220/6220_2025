// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.V2_SparkMaxWristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class wristUpOneDegree extends Command {
  V2_SparkMaxWristSubsystem wrist = V2_SparkMaxWristSubsystem.getInstance();
  ElevatorSubsystem elev = ElevatorSubsystem.getInstance();
  public wristUpOneDegree() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist, elev);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    System.out.println("wrist up init");
    wrist.setGoal(wrist.getGoalPosition()+1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    elev.driveToGoal();
    wrist.driveToGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    elev.stop();
    wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
