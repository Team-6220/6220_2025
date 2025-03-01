// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.core.CoreCANrange;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANrangeConfiguration;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import com.ctre.phoenix6.hardware.CANrange;

public class CANRangeSubsystem extends SubsystemBase{
    private final double distanceThreshold1 = 10.0; // Change distance
    private final double distanceThreshold2 = 15.0; // Change distance
    private static CANRangeSubsystem INSTANCE = null;
    private CANrange cRange1, cRange2, cRange3;
    // private double distance1, distance2, distance3;
    // private StatusSignal<Distance> distance1, distance2, distance3;
    private final CANrangeConfiguration configs = new CANrangeConfiguration();
    
    
    public CANRangeSubsystem() {
      cRange1 = new CANrange(10); // Change deviceID
      cRange2 = new CANrange(11); // Change deviceID
      cRange3 = new CANrange(12); // Change deviceID

      cRange1.getConfigurator().apply(configs); // wrist can range
      cRange2.getConfigurator().apply(configs); // front intake can range 1
      cRange3.getConfigurator().apply(configs); // other front intake can range (2)
    }
    
    // public void updDistance() {
    //   distance1 = cRange1.getDistance();
    //   distance2 = cRange2.getDistance();
    //   distance3 = cRange3.getDistance();
    // }

    /**
     * Checks if an object is within the 10mm range.
     * @return true if object is detected within range, false otherwise.
     */
    public boolean isObjectInWrist() {
      return cRange1.getDistance().getValueAsDouble() <= distanceThreshold1;
    }

    public boolean isObjectInFrontIntake() {
      return (cRange2.getDistance().getValueAsDouble() <= distanceThreshold2 || (cRange3.getDistance().getValueAsDouble() <= distanceThreshold2));
    }
    
    public CANrange getCANRange1() { 
      return cRange1;
    }

    public CANrange getCANRange2() { 
      return cRange2;
    }

    public CANrange getCANRange3() { 
      return cRange3;
    }

    @Override
    public void periodic() {
      // updDistance();
      SmartDashboard.putBoolean("Can Range Wrist", isObjectInWrist());
      SmartDashboard.putBoolean("Can Range Front Intake", isObjectInFrontIntake());
    }
    
    public static synchronized CANRangeSubsystem getInstance() {
      if (INSTANCE == null) {
      INSTANCE = new CANRangeSubsystem();
      }
    return INSTANCE;
    }

    /**
    * Interface to simulate sensor interaction. Replace with actual implementation.
    */
    interface SensorInterface {
      double getDistance(); // Returns the measured distance in mm
    }
}

