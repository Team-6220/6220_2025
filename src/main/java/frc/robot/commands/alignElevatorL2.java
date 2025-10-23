// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.ElevatorSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class alignElevatorL2 extends ParallelCommandGroup {
  /** Creates a new alignElevatorL4. */
  photonAlignCmd autonAlign;
  Stage4CMD autonL2;
  ElevatorSubsystem elevator;
  public alignElevatorL2() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // autonAlign = new photonAlignCmd(0, null, 0, 0);
    autonL2 = new Stage4CMD(true);
    elevator = ElevatorSubsystem.getInstance();
    addCommands(autonAlign, autonL2);
  }

  public BooleanSupplier isDone() {
    BooleanSupplier temp = () -> (autonAlign.isFinished() && elevator.elevatorAtGoal());
    return temp;
  }
}
