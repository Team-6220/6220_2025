// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.frontIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class testcommand extends Command {
  private frontIntakeSubsystem m_fiss;
  double a;
  public testcommand(frontIntakeSubsystem s_fiss) {
    m_fiss=s_fiss;
    addRequirements(s_fiss);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    a= m_fiss.getElevatorPosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_fiss.spinFront(true);
    m_fiss.swingToGoal(a);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
