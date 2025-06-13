// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.AutoConstants;
import frc.robot.VisionConstants;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.Stage2CMD;
import frc.robot.commands.photonAlignCmd;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class blueAlignBThenScoreL2 extends SequentialCommandGroup {
  /** Creates a new blueAlignBThenScoreL2. */
  public blueAlignBThenScoreL2(Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Distance blueBX = Meters.of(2.83);
    Distance blueBY = Meters.of(3.84);
    Angle blueBangle = Degrees.of(0);
      addCommands(
        AutoBuilder.pathfindToPose(new Pose2d(2.5, 4.0, new Rotation2d(Degrees.of(0))), AutoConstants.pathConstraints),
        new photonAlignCmd(0, swerve, VisionConstants.rightReefX, VisionConstants.rightReefY),
        new InstantCommand(() -> swerve.setPose(new Pose2d(blueBX, blueBY, new Rotation2d(blueBangle)))),
        new Stage2CMD(true),
        new EjectCoral()
        );
      }
}
