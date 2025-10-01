// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ledManager extends SubsystemBase {
  public TOFSubsystem tofsub;
  public LEDCANdle ledcan;
  public GenericHID buttonBoard;
  public boolean hasit=false;
  public ledManager(TOFSubsystem p_tofsub, LEDCANdle p_ledcan, GenericHID p_buttonboard) {
    this.tofsub = p_tofsub;
    this.ledcan = p_ledcan;
    this.buttonBoard=p_buttonboard;
  }

  @Override
  public void periodic() {
    //logic for getting canrange stuff and calling led candle
    //should be very simple
    //for hasit make a threshold with canrange
    if()

    if((buttonBoard.getRawButtonPressed(-1))&&!hasit){//input buttons
      ledcan.setRed();
    }
    else if(hasit){
      ledcan.setGreen();
    }
    else{
      ledcan.setGold();
    }
  }
}
