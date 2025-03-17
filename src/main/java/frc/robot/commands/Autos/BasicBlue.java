// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import frc.robot.Constants.AutoConstants;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CoralStationCmd;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.Stage2CMD;
import frc.robot.commands.Stage3CMD;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BasicBlue extends SequentialCommandGroup {
  /** Creates a new BasicBlue. */
  Swerve s_swerve;
  public BasicBlue(Swerve s_swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.s_swerve = s_swerve;
    addCommands(
      new InstantCommand(() -> s_swerve.setPose(AutoConstants.startPosesBlue[4])),//0 - outter most, 1 - less outter most, 2 - center
      AutoBuilder.pathfindToPose(AutoConstants.waypointPosesBlue[2], AutoConstants.pathConstraints),//pathfind
       new Stage3CMD( 0, new Trigger(() -> true), new Trigger(() -> false)), //either Stage2CMD or Stage3CMD; Stage4CMD not finished
      new EjectCoral(),
      AutoBuilder.pathfindToPose(AutoConstants.waypointPosesBlue[6], AutoConstants.pathConstraints), //6 for right, 7 for left
      new CoralStationCmd(),
      AutoBuilder.pathfindToPose(AutoConstants.waypointPosesBlue[1], AutoConstants.pathConstraints),
      new Stage3CMD(0, new Trigger(() -> true), new Trigger(() -> false)), //either Stage2CMD or Stage3CMD; Stage4CMD not finished
      new EjectCoral()
    );
  }
}
