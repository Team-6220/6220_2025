// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.LEDCANdle;
import frc.robot.subsystems.V2_SparkMaxWristSubsystem;
import frc.lib.util.TunableNumber;
import frc.robot.ElevatorConstants;
import frc.robot.WristConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Stage2CMD extends Command {
  private ElevatorSubsystem elevator;
  private V2_SparkMaxWristSubsystem wrist;

  private TunableNumber elevHeight = new TunableNumber("l2 elev height", ElevatorConstants.E_L2);
  private TunableNumber wristDegrees = new TunableNumber("l2 wrist", WristConstants.L2);

  // private LEDCANdle candle;

  private int autoCounter = 0;
  private boolean isAuto;

  public Stage2CMD(boolean isAuto) {
    elevator = ElevatorSubsystem.getInstance();
    // candle = LEDCANdle.getInstance();
    wrist = V2_SparkMaxWristSubsystem.getInstance();
    autoCounter = 0;
    this.isAuto = isAuto;
    addRequirements(elevator);
    addRequirements(wrist);
    // addRequirements(candle);
    // addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setGoal(ElevatorConstants.E_L2);
    wrist.setGoal(WristConstants.L2);
    // candle.setColor(255, 255, 0, 30, 8, 100);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // elevator.driveToGoal(ElevatorConstants.L2HeightRaw);
    if (elevHeight.hasChanged()) {
      elevator.setGoal(elevHeight.get());
    }
    if (wristDegrees.hasChanged()) {
      wrist.setGoal(wristDegrees.get());
    }
    wrist.driveToGoal();
    elevator.driveToGoal();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("we ended");
    elevator.stop();
    wrist.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (isAuto && (Math.abs(elevator.getElevatorPositionMeters() - elevator.getGoal()) < 0.05)) {
      autoCounter++;
    }
    if (autoCounter >= 15) {
      return true;
    }
    return false;
  }
}
