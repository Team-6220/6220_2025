// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.FrontIntakeConstants;
import frc.robot.subsystems.frontIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeGround extends Command {
  private frontIntakeSubsystem m_fiss = frontIntakeSubsystem.getInstance();
  private TunableNumber lowergroundsetpoint=new TunableNumber("lower intake ground setpoint", FrontIntakeConstants.intakeCoralSetpoint);
  public IntakeGround() {
    addRequirements(m_fiss);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_fiss.setGoal(lowergroundsetpoint.get());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if(lowergroundsetpoint.hasChanged())
    {
      m_fiss.setGoal(lowergroundsetpoint.get());
    }
    m_fiss.swingToGoal();
    m_fiss.setFront(FrontIntakeConstants.wheelSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_fiss.setFront(FrontIntakeConstants.idleSpinVoltage);
    m_fiss.setGoal(FrontIntakeConstants.idleSetpoint);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
