// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.V2_SparkMaxWristSubsystem;
import frc.lib.util.TunableNumber;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.WristConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Stage4CMD extends Command
{
   private ElevatorSubsystem elevator;
  private V2_SparkMaxWristSubsystem wrist;
  
  private TunableNumber elevHeight = new TunableNumber("l4 elev height", ElevatorConstants.E_L4);
  private TunableNumber wristDegrees = new TunableNumber("l4 wrist", WristConstants.L4);

  public Stage4CMD()
  {
    elevator = ElevatorSubsystem.getInstance();
    addRequirements(elevator);
    wrist = V2_SparkMaxWristSubsystem.getInstance();
    addRequirements(wrist);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    wrist.setGoal(WristConstants.L4);
    elevator.setGoal(ElevatorConstants.E_L4);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()  {
    if(elevHeight.hasChanged())
    {
      elevator.setGoal(elevHeight.get());
    }
    if(wristDegrees.hasChanged())
    {
      wrist.setGoal(wristDegrees.get());
    }
    elevator.driveToGoal();
    wrist.driveToGoal();
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
  public boolean isFinished()
  {
    return false;
  }
}
