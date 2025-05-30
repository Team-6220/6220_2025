// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.VisionConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class alignAndScoreRightL2 extends ParallelDeadlineGroup {
  /** Creates a new alignThenScoreL2. */
  public alignAndScoreRightL2() {
    // Add the deadline command in the super() call. Add other commands using
    // addCommands().
    super(new Stage2CMD(true));
      addCommands(new photonAlignCmd(0, Swerve.getInstance(), VisionConstants.rightReefX, VisionConstants.rightReefY));
    // addCommands(new FooCommand(), new BarCommand());
  }
}
