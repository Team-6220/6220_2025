// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.CoralStationCmd;
import frc.robot.commands.EjectCoral;
import frc.robot.commands.IntakeCoral;
import frc.robot.commands.IntakeGround;
import frc.robot.commands.Stage2CMD;
import frc.robot.commands.Stage3CMD;
import frc.robot.commands.Stage4CMD;
import frc.robot.commands.OutakeLowerIntake;
//import frc.robot.commands.Autos;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.lowerIntakeAlgeaPickUp;
import frc.robot.commands.lowerIntakeSet;
import frc.robot.commands.ElevatorManuel;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.V2_SparkMaxWristSubsystem;
import frc.robot.subsystems.frontIntakeSubsystem;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  

  private final Swerve s_Swerve = new Swerve();

  private final ElevatorSubsystem elevator = ElevatorSubsystem.getInstance();
  private final frontIntakeSubsystem frontIntake = frontIntakeSubsystem.getInstance();

  private final CommandXboxController m_driverController =
      new CommandXboxController(0);

  private final Joystick m_joystick = new Joystick(1);

  private final GenericHID m_buttonBoard = new GenericHID(2);

  private final Trigger stage2 = new Trigger(() -> m_buttonBoard.getRawButton(5));
  private final Trigger stage3 = new Trigger(() -> m_buttonBoard.getRawButton(3));
  private final Trigger stage4 = new Trigger(() -> m_buttonBoard.getRawButton(1));
  private final Trigger coralStation = new Trigger(() -> m_buttonBoard.getRawButton(2));
  private final Trigger elevatorIntake = new Trigger(() -> m_joystick.getRawButton(1));
  private final Trigger elevatorOuttake = new Trigger(() -> m_joystick.getRawButton(2));
  private final Trigger resetEncoder = new Trigger(() -> m_buttonBoard.getRawButton(11));
  private final Trigger elevatorUp = new Trigger(() -> m_buttonBoard.getRawButton(13));
  private final Trigger elevatorDown = new Trigger(() -> m_buttonBoard.getRawButton(14));
  private final Trigger groundIntake = new Trigger(() -> m_buttonBoard.getRawButton(15));
  private final Trigger setLowerIn = new Trigger(() -> m_buttonBoard.getRawButton(4));
  private final Trigger lowerOuttake = new Trigger(() -> m_buttonBoard.getRawButton(6));

  private final Trigger test = new Trigger(() -> m_joystick.getRawButton(5));
  private final Trigger twisterTest = new Trigger(() -> m_buttonBoard.getRawButton(22));//turn right
  // private final Trigger lowerIntake = new Trigger(() -> m_buttonBoard.getRawButton(4));
  // private final Trigger lowerOuttake = new Trigger(() -> m_buttonBoard.getRawButton(6));


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    s_Swerve.zeroHeading(m_driverController.getHID());

    //s_Swerve.configureAutoBuilder();

    elevator.setDefaultCommand(
      new ElevatorManuel(m_joystick)
    );

    // frontIntake.setDefaultCommand(new lowerIntakeSet());

    // frontIntake.setDefaultCommand(new LowerIntakeManual(m_joystick));

    // wrist.setDefaultCommand(
      // new wristTest(m_driverController.getHID())
    // );

    s_Swerve.setDefaultCommand(
        new TeleopSwerve(
            s_Swerve,
            m_driverController,
            m_driverController.leftBumper())
        );

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Auto Chooser", autoChooser);
    //TODO: Register named commands as needed
    //NamedCommands.registerCommand(null, null);

    configureBindings();
    
  }

  

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    m_driverController.y().onTrue(new InstantCommand(() -> s_Swerve.zeroHeading(m_driverController.getHID())));

    resetEncoder.onTrue(new InstantCommand(() -> elevator.resetEncoder()));
    stage2.onTrue(new Stage2CMD());
    stage3.onTrue(new Stage3CMD());
    stage4.onTrue(new Stage4CMD());
    
    coralStation.onTrue(new CoralStationCmd());
    elevatorIntake.whileTrue(new IntakeCoral());
    elevatorOuttake.whileTrue(new EjectCoral());
    groundIntake.whileTrue(new IntakeGround());
    setLowerIn.whileTrue(new lowerIntakeAlgeaPickUp());
    setLowerIn.whileFalse(new lowerIntakeSet());
    lowerOuttake.whileTrue(new OutakeLowerIntake());

    twisterTest.onTrue(new InstantCommand(()-> System.out.println("^^^^^^^^^^^pressed")));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new Command() {
      
    };
      
  }
}