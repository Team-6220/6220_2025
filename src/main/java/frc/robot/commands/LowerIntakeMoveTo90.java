// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.frontIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class LowerIntakeMoveTo90 extends Command {
  /** Creates a new LowerIntakeMoveTo90. */
  private frontIntakeSubsystem frontIntake;

  public LowerIntakeMoveTo90() {
    // Use addRequirements() here to declare subsystem dependencies.
      frontIntake = frontIntakeSubsystem.getInstance();
      addRequirements(frontIntake);
  }
  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    frontIntake.setGoal(90);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    frontIntake.swingToGoal();
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
