// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import static edu.wpi.first.units.Units.Degree;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.DegreesPerSecond;
import static edu.wpi.first.units.Units.DegreesPerSecondPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.MetersPerSecondPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestingAutoRed extends SequentialCommandGroup {
  
  /** Creates a new TestingAuto. */
  public TestingAutoRed(Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Swerve s_Swerve = swerve;
    double x = 14.124, y = 2.395, theta = 0;
    addCommands(
      new InstantCommand(() -> s_Swerve.setPose(new Pose2d(new Translation2d(14.65, 4.075), new Rotation2d(Degrees.of(0))))),
      AutoBuilder.pathfindToPose(new Pose2d(x, y, new Rotation2d(Degrees.of(theta))), AutoConstants.pathConstraints)
    );
  }
}
