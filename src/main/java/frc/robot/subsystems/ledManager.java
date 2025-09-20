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
  public ledManager(TOFSubsystem p_tofsub, LEDCANdle p_ledcan, GenericHID p_buttonboard) {
    this.tofsub = p_tofsub;
    this.ledcan = p_ledcan;
    this.buttonBoard=p_buttonboard;
  }

  @Override
  public void periodic() {
    //logic for getting canrange stuff and calling led candle
    ledcan.setBlue();
    ledcan.setGold();
    ledcan.setRed();
    ledcan.setGreen();
    //should be very simple
    if(buttonBoard.getRawButtonPressed(-1)){}
  }
}
