// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CANRangeSubsystem;
import frc.robot.subsystems.WristIntakesubsytem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class EjectCoral extends Command {
  /** Creates a new EjectCoralTest. */
  WristIntakesubsytem wristIntake = WristIntakesubsytem.getInstance();
  CANRangeSubsystem range = CANRangeSubsystem.getInstance();
  boolean isAuto;

  public EjectCoral() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wristIntake);
    isAuto = false;
  }

  public EjectCoral(boolean isAuto) {
    this.isAuto = isAuto;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wristIntake.ejectCoral();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // wristIntake.simpleDrive(interrupted, 0);
    wristIntake.endOccupied();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isAuto && range.isObjectInWrist()) {
      return true;
    }
    return false;
  }
}
