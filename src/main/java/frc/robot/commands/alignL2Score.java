// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class alignL2Score extends SequentialCommandGroup {
  /** Creates a new alignL4Score. */
  alignElevatorL2 align;
  Swerve swerve;
  public alignL2Score(Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.swerve = swerve;
    align = new alignElevatorL2(swerve);
    
    addCommands(align.until(align.isDone()), new ParallelDeadlineGroup (new WaitCommand(2), new EjectCoral(true),new Stage2CMD(true)));
  }
}
