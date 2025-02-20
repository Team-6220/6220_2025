// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.frontIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class lowerintaketestcommand extends Command {
  private frontIntakeSubsystem m_fiss = frontIntakeSubsystem.getInstance();
  private CommandXboxController m_driverController;
  private double a;

  public lowerintaketestcommand(CommandXboxController driver) {
    m_driverController = driver;
    a = m_fiss.getPosition();
    addRequirements(m_fiss);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_fiss.setGoal(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(m_driverController.a().getAsBoolean()){}
    //m_fiss.simpleintakeDrive(0.25);
    // if(m_driverController.y().getAsBoolean()){
      // m_fiss.spinFront(true, false);}
    m_fiss.simpleDrive(m_driverController.getLeftY()); //range 0.67 - 0.23
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_fiss.simpleintakeDrive(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
