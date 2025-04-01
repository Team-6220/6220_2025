// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.LEDCANdle;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CandleCommand extends Command {
  LEDCANdle m_LEDCANdle;
  CommandXboxController m_driverController;
  String mode;

  public CandleCommand(CommandXboxController driver, String m) {
    m_LEDCANdle = LEDCANdle.getInstance();
    m_driverController = driver;
    mode = m;
    addRequirements(m_LEDCANdle);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mode.equals("rainbow")) {
      if (m_driverController.leftBumper().getAsBoolean()) {
        m_LEDCANdle.setRai(false);
      }
      if (m_driverController.rightBumper().getAsBoolean()) {
        m_LEDCANdle.setRai(true);
      }
    }
    if (mode.equals("testing")) {
      if (m_driverController.leftBumper().getAsBoolean()) {
        m_LEDCANdle.setTesting(false);
      }
      if (m_driverController.rightBumper().getAsBoolean()) {
        m_LEDCANdle.setTesting(true);
      }
    }
    if (mode.equals("error")) {
      m_LEDCANdle.setError();
    }
    /*if(mode.equals("blinking")){
      m_LEDCANdle.setBlinking();
    } */
    if (mode.equals("adj")) {
      m_LEDCANdle.setModifiable((int) ((m_driverController.getLeftX() + 1) * 127));
      System.out.println((int) ((m_driverController.getLeftX() + 1) * 127));
    }
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
