// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.subsystems;

// import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import edu.wpi.first.wpilibj.Timer;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.networktables.NetworkTable;
// import edu.wpi.first.networktables.NetworkTableEntry;
// import edu.wpi.first.networktables.NetworkTableInstance;


// public class timerSubsystem extends SubsystemBase {
//   private static final String TEAM_IP = "10.62.20.2";
//   private final NetworkTableInstance ntInstance;
//   private final NetworkTable fmsTable;
//   private final NetworkTableEntry matchTimeEntry;

//   /** Creates a new timerSubsystem. */
//   public timerSubsystem() {
//     ntInstance = NetworkTableInstance.getDefault();
//     ntInstance.startClient4(TEAM_IP); // Connect to RoboRIO/FMS
//     System.out.println("Connected to NetworkTables at " + TEAM_IP);

//     fmsTable = ntInstance.getTable("FMSInfo");
//     matchTimeEntry = fmsTable.getEntry("MatchTime");
//   }


//   @Override
//   public void periodic() {
//     // This method will be called once per scheduler run
//     double matchTime = matchTimeEntry.getDouble(0); // Default 0 if no data
//     SmartDashboard.putNumber("Remaining Match Time", matchTime); // Display on SmartDashboard
//     // System.out.println("Match Time: " + matchTime + "s"); // Print for debugging
//   }
// }
