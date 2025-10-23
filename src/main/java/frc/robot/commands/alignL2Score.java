// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class alignL2Score extends SequentialCommandGroup {
  /** Creates a new alignL4Score. */
  alignElevatorL2 align;
  public alignL2Score() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    align = new alignElevatorL2();
    
    addCommands(align.until(align.isDone()), new EjectCoral(true));
  }
}
