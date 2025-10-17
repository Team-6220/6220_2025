// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ledManager extends SubsystemBase {
  public CANRangeSubsystem tofsub;
  public LEDCANdle ledcan;
  public GenericHID buttonBoard;
  public Joystick joystick;
  //public boolean hasit=false;
  public ledManager(CANRangeSubsystem p_tofsub, LEDCANdle p_ledcan, GenericHID p_buttonboard, Joystick joystick) {
    this.tofsub = p_tofsub;
    this.ledcan = p_ledcan;
    this.buttonBoard=p_buttonboard;
    this.joystick = joystick;
  }

  @Override
  public void periodic() {
    //logic for getting canrange stuff and calling led candle
    //should be very simple
    //for hasit make a threshold with canrange
    if((buttonBoard.getRawButtonPressed(2)||buttonBoard.getRawButtonPressed(15)||joystick.getRawButtonPressed(1))&&!(tofsub.isObjectInFrontIntake()||tofsub.isObjectInWrist())){//input buttons
      ledcan.setRed();
    }
    else if((buttonBoard.getRawButtonPressed(8)||buttonBoard.getRawButtonPressed(6)||joystick.getRawButtonPressed(2))){
      ledcan.setBlue();
    }
    else if(tofsub.isObjectInFrontIntake()||tofsub.isObjectInWrist()){
      ledcan.setGreen();
    }
    else{
      ledcan.setGold();
    }
  }
}
