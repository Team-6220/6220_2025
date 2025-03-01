// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.IFollower;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class buttonLogic extends Command {
  /** Creates a new buttonLogic. */
  private boolean io = true; // intake if true, outake if false

  public void IntakeButton(){
    io = true;
  }

  public void OuttakeButton(){
    io = false;
  }

  public void L1Button(){
    if (io) {
      //intake functionality
    }
    else{
      //outtake functionality
    }
  }


  public void L2Button(){
    if (io) {
      //intake functionality
    }
    else{
      //outtake functionality
    }
  }


  public void L3Button(){
    if (io) {
      //intake functionality
    }
    else{
      //outtake functionality
    }
  }

  public void L4Button(){
    if (io) {
      //intake functionality
    }
    else{
      //outtake functionality
    }
  }

  public void CButton(){
    if (io) {
      //intake functionality
    }
    else{
      //outtake functionality
    }
  }

  public void GCButton(){
    if (io) {
      //intake functionality
    }
    else{
      //outtake functionality
    }
  }

  public void GAButton(){
    if (io) {
      //intake functionality
    }
    else{
      //outtake functionality
    }
  }










  public buttonLogic() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
