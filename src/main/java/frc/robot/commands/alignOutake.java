// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import frc.robot.subsystems.CANRangeSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class alignOutake extends ParallelDeadlineGroup {
  /** Creates a new alignOutake. */
  CANRangeSubsystem range = CANRangeSubsystem.getInstance();
  EjectCoral eject= new EjectCoral();
  public alignOutake() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new EjectCoral());
    addCommands(new Stage2CMD(true));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
