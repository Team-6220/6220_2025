// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.PhotonVisionSubsystem;
import frc.robot.subsystems.V2_SparkMaxWristSubsystem;
import frc.lib.util.TunableNumber;
import frc.robot.Constants;
import frc.robot.ElevatorConstants;
import frc.robot.VisionConstants;
import frc.robot.WristConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Stage3CMD extends Command {
  private ElevatorSubsystem elevator;
  private V2_SparkMaxWristSubsystem wrist;

  private TunableNumber elevHeight = new TunableNumber("l3 elev height", ElevatorConstants.E_L3);
  private TunableNumber wristDegrees = new TunableNumber("l3 wrist", WristConstants.L3);

  private int autoCounter = 0;
  private boolean isAuto;

  public Stage3CMD(boolean isAuto) {
    elevator = ElevatorSubsystem.getInstance();
    wrist = V2_SparkMaxWristSubsystem.getInstance();

    autoCounter = 0;
    this.isAuto = isAuto;
    // this.s_Swerve = s_Swerve;
    // addRequirements(s_Photon);
    addRequirements(elevator);
    addRequirements(wrist);
    // addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setGoal(elevHeight.getDefault());
    wrist.setGoal(wristDegrees.getDefault());
    VisionConstants.setTagXYHeightAngle();
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
    // s_Swerve.stopDriving();
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
