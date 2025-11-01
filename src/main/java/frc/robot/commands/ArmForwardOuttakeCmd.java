// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.FrontIntakeConstants;
import frc.robot.subsystems.frontIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ArmForwardOuttakeCmd extends Command {
  /** Creates a new ArmForwardOuttakeCmd. */
  frontIntakeSubsystem frontintake = frontIntakeSubsystem.getInstance();
  public ArmForwardOuttakeCmd() {
    addRequirements(frontintake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    frontintake.resetPID();
    frontintake.setGoal(FrontIntakeConstants.frontOuttakeAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    frontintake.swingToGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    frontintake.simpleDrive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
