// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TOFSubsystem extends SubsystemBase {
  public final static int lowerleftRangeID=22;
  public final static int lowerrightRangeID=23;
  public final static int topRangeID=21;
  public static CANrange lowerleft;
  public static CANrange lowerright;
  public static CANrange top;
  public TOFSubsystem() {
    lowerleft = new CANrange(lowerleftRangeID);
    lowerright = new CANrange(lowerrightRangeID);
    top = new CANrange(topRangeID);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("lowerleftCanrange", lowerleft.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("lowerrightCanrange", lowerright.getDistance().getValueAsDouble());
    SmartDashboard.putNumber("topCanrange", top.getDistance().getValueAsDouble());
  }
  public double getLowerDistance(){
    return top.getDistance().getValueAsDouble();
  }
  public double getElevatorDistance(){
    return Math.min(lowerright.getDistance().getValueAsDouble(), lowerleft.getDistance().getValueAsDouble());
  }
}
