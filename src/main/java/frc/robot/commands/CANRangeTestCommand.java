// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANRangeSubsystem;
import frc.robot.subsystems.V2_SparkMaxWristSubsystem;
import frc.robot.subsystems.WristIntakesubsytem;
import frc.robot.subsystems.frontIntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CANRangeTestCommand extends Command {
  /** Creates a new CANRangeTestCommand. */
  private V2_SparkMaxWristSubsystem wrist;
  private CANRangeSubsystem cRange;
  private frontIntakeSubsystem frontIntake;

  public CANRangeTestCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
    wrist = V2_SparkMaxWristSubsystem.getInstance();
    addRequirements(wrist);
    cRange = CANRangeSubsystem.getInstance();
    addRequirements(cRange);
    frontIntake = frontIntakeSubsystem.getInstance();
    addRequirements(frontIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(cRange.isObjectInWrist())
    {
      wrist.stop();
      // turn on LEDS
    }
    
    if (cRange.isObjectInFrontIntake()) {
      frontIntake.spinFront(false, false);
      // turn on LEDS
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
