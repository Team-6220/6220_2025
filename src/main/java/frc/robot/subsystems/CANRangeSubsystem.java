// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

package com.ctre.phoenix6.hardware.core


import com.ctre.phoenix6.StatusSignal;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CANRangeSubsystem extends SubsystemBase {
  /** Creates a new CANRangeSubsystem. */
  private double distance = 2;
  public CANRangeSubsystem() {
    
    // if loaded with coral, it is within range (distance)
    // check it with distance
    // return true for led to turn on
    if(getDistance ){

    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
