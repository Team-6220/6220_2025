// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands;

// import com.ctre.phoenix6.hardware.CANrange;

// import edu.wpi.first.wpilibj2.command.Command;
// import frc.robot.subsystems.CANRangeSubsystem;
// import frc.robot.subsystems.V2_SparkMaxWristSubsystem;
// import frc.robot.subsystems.WristIntakesubsytem;
// import frc.robot.subsystems.frontIntakeSubsystem;
// import java.util.function.BooleanSupplier;
// // import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.robot.subsystems.LEDCANdle;

// /* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
// public class CANRangeTestCommand extends Command {
//   /** Creates a new CANRangeTestCommand. */
//   // private V2_SparkMaxWristSubsystem wrist;
//   // private CANRangeSubsystem cRange;
//   // private frontIntakeSubsystem frontIntake;
//   LEDCANdle m_LEDCANdle;

//   public CANRangeTestCommand() {
//     // Use addRequirements() here to declare subsystem dependencies.
//     // wrist = V2_SparkMaxWristSubsystem.getInstance();
//     // addRequirements(wrist);
//     cRange = CANRangeSubsystem.getInstance();
//     addRequirements(cRange);
//     // frontIntake = frontIntakeSubsystem.getInstance();
//     // addRequirements(frontIntake);
//     m_LEDCANdle = LEDCANdle.getInstance();
//     addRequirements(m_LEDCANdle);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {}

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if(cRange.isObjectInWrist() && cRange.isObjectInFrontIntake())
//     {
//       m_LEDCANdle.setGreen();
//     }
//     else if(cRange.isObjectInWrist())
//     {
//       // wrist.stop();
//       m_LEDCANdle.setColor(0,128,128,105,0,308);
//     }
//     else if (cRange.isObjectInFrontIntake()) {
//       // frontIntake.spinFront(false, false);
//       m_LEDCANdle.setBlue();
//     }

//     if (!(cRange.isObjectInWrist() && cRange.isObjectInFrontIntake())) {
//       m_LEDCANdle.setRed();
//     }
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {

//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
