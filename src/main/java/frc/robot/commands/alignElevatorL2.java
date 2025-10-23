// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.VisionConstants;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class alignElevatorL2 extends ParallelCommandGroup {
  /** Creates a new alignElevatorL4. */
  photonAlignCmd autonAlign;
  Stage4CMD autonL2;
  ElevatorSubsystem elevator;
  Swerve swerve;
  public alignElevatorL4(Swerve swerve) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    this.swerve = swerve;
    autonAlign = new photonAlignCmd(0, swerve, VisionConstants.leftReefX, VisionConstants.leftReefY);
    autonL4 = new Stage4CMD(true);
    elevator = ElevatorSubsystem.getInstance();
    addCommands(autonAlign, autonL2);
  }

  public BooleanSupplier isDone() {
    BooleanSupplier temp = () -> (autonAlign.isFinished() && elevator.elevatorAtGoal());
    return temp;
  }
}
