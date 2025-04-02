// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.V2_SparkMaxWristSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ElevatorManuel extends Command {
  /** Creates a new ElevatorManuel. */
  ElevatorSubsystem elevSub = ElevatorSubsystem.getInstance();

  V2_SparkMaxWristSubsystem wrist = V2_SparkMaxWristSubsystem.getInstance();
  Joystick m_joystick;

  public ElevatorManuel(Joystick m_joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_joystick = m_joystick;
    addRequirements(elevSub, wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double output =
        -m_joystick
            .getY(); // flipped the sign because in the joystick up is negative but elevator up is
                     // positive
    if (elevSub.getElevatorPositionMeters() > ElevatorConstants.upperEncoderExtreme && output > 0) {
      output = 0;
    }

    // YOU HAVE TO MANUELLY RESET IT -- we tried manuel reset but it's too risky because we might
    // bend a shaft
    // If we go too low/high and bend the shaft it will affect our PID values and WE DON'T WANT THAT
    if (elevSub.getElevatorPositionMeters() < ElevatorConstants.lowerEncoderExtreme && output < 0) {
      output = 0;
    }
    elevSub.simpleDrive(output);
    wrist.driveToGoal();
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
