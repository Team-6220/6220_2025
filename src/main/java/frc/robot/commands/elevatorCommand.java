// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.Constants.ElevatorConstants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class elevatorCommand extends Command
{
  private ElevatorSubsystem elevator;
  private GenericHID driver;

  public elevatorCommand(GenericHID driver)
  {
    elevator = ElevatorSubsystem.getInstance();
    this.driver = driver;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    //We're not scoring on stage 1, the wood shaft
    int stage = 0;
    if (driver.getRawButtonPressed(1))
    {
      stage = 4;
    } 
    else if (driver.getRawButtonPressed(2))
    {
      stage = 3;
    }
    else if (driver.getRawButtonPressed(3))
    {
      stage = 2;
    }

    if (stage == 4)
    {
      // add actual values
      elevator.driveToGoal(ElevatorConstants.L4HeightRaw);
    }
    if (stage == 3)
    {
      // add actual values
      elevator.driveToGoal(ElevatorConstants.L3HeightRaw);
    }
    if (stage == 2)
    {
      // add actual values
      elevator.driveToGoal(ElevatorConstants.L2HeightRaw);
    }
    if (stage == 0)
    {
      elevator.driveToGoal(ElevatorConstants.L2HeightRaw); //prevent the pivet bumping into the wooden thign when it extends.
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()  {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return false;
  }
}
