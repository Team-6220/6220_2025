// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.KilogramSquareMeters;
import static edu.wpi.first.units.Units.Pound;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Optional;

import org.opencv.core.Mat.Tuple2;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.path.PathConstraints;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MomentOfInertia;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.XboxController;
import frc.lib.util.AlienceColorCoordinateFlip;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;
import frc.lib.util.TunableNumber;

public final class Constants {

    public final TunableNumber autoMaxVelocity = new TunableNumber("autoMaxVelocity", AutoConstants.autoMaxVelocityMps);
    public final TunableNumber autoMaxAcceleratMpsSq = new TunableNumber("autoMaxAcceleratMpsSq", AutoConstants.autoMaxAcceleratMpsSq);
    public final TunableNumber maxAngularVelocityRps = new TunableNumber("maxAngularVelocityRps", AutoConstants.maxAngularVelocityRps);
    public final TunableNumber maxAngularAcceleratRpsSq = new TunableNumber("maxAngularAcceleratRpsSq", AutoConstants.maxAngularAcceleratRpsSq);
    
    public static boolean TUNING_MODE = true;

    public static String isRed = "N/A";

    public static final Mass robotMass = Pound.of(100);
    public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(4.563);

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final double kDeadband = 0.065;

        public static final int translationAxis = XboxController.Axis.kLeftY.value;
        public static final int strafeAxis = XboxController.Axis.kLeftX.value;
        public static final int rotationAxis = XboxController.Axis.kRightX.value;

        /*Start from zero after dead band.
         * Eg. if your deadband is .05
         *     if you don't have this function the minimum input would be .05
         *     if you use this function the input would be 0 when the joystick reading is at .05
        */
        public static double modifyMoveAxis(double value) {
            // Deadband
            if(Math.abs(value) < kDeadband) {
                return 0;
            }
        
            // Change the axis
            double b = .1;
            double a = .5;
            double x = value;
            if(value >=0) {
                return b + (1-b)*(a*Math.pow(x, 3) + (1-a)*x);
            } else{
                return -b + (1-b)*(a*Math.pow(x, 3) + (1-a)*x);
            }
            //value = Math.copySign(value * value, value);
        
            //return value;
          }
        public static double modifyRotAxis(double value) {
            // Deadband
            if(Math.abs(value) < kDeadband) {
                return 0;
            }
        
            // Change the axis
            double b = .05;
            double a = .2;
            if(value >=0) {
                return b + (1-b)*(a*Math.pow(value, 3) + (1-a)*value);
            } else{
                return -b + (1-b)*(a*Math.pow(value, 3) + (1-a)*value);
            }
          }

        public static double[] getDriverInputs(XboxController driver) {
            double[] inputs = new double[3];

            inputs[0] = OIConstants.modifyMoveAxis(-driver.getRawAxis(translationAxis));
            inputs[1] = OIConstants.modifyMoveAxis(-driver.getRawAxis(strafeAxis));
            inputs[2] = OIConstants.modifyRotAxis(-driver.getRawAxis(rotationAxis));

            inputs[0] = MathUtil.applyDeadband(inputs[0], OIConstants.kDeadband);
            inputs[1] = MathUtil.applyDeadband(inputs[1], OIConstants.kDeadband);
            inputs[2] = MathUtil.applyDeadband(inputs[2], OIConstants.kDeadband);

            int invert =  (Constants.isRed.equals("red")) ? -1 : 1; 

            inputs[0] *= invert;
            inputs[1] *= invert;

            inputs[0] *= SwerveConstants.maxSpeed;
            inputs[1] *= SwerveConstants.maxSpeed;
            inputs[2] *= SwerveConstants.maxAngularVelocity;
            return inputs;
        }
    }

    public static final class EWIConstnats{
        /*Elevator START*/
        public static final int elevatorMotorAID = 13;//TODO: need to be changed 2025
        public static final int elevatorMotorBID = 14;//TODO: need to be changed 2025

        public static final boolean elevatorMotorAInverted = true;//TODO: need to be changed 2025
        public static final boolean elevatorMotorBInverted = true;//TODO: need to be changed 2025

        //TODO: need to be changed 2025 (all of the pid values)
        public static final double kP = 0.3; //0.009
        public static final double kI = 0.1;//0.0005
        public static final double kD = 0.005;//0.001
        public static final double kG = 0.37;//Feedforward
        public static final double kV = 0.025;//Feedforward
        public static final double kS = 0.45;//Feedforward
        public static final double elevatorMaxVel = 200;//not sure if we need this
        public static final double elevatorMaxAccel = 450;//not sure if we need this

        public static final double minElevatorEncoderReading = 0;//TODO: need to be changed 2025
        public static final double maxElevatorEncoderReading = 100;//TODO: need to be changed 2025
        //NOTE: this is an alternative for the elvatorL1-L4 & intake
        public static final double [] elevatorLookUpTable = {//TODO: need to be changed 2025
            70,//L1
            70,//L2
            70,//L3
            70,//L4
            70,//Intake
        };

        public static final double elevatorOffset = 167.53781218844532;//not sure if we need this//TODO: need to be changed 2025
        //FIXME: set setpoints
        public static final double intakeElevatorSetpoint = 84;//TODO: need to be changed 2025
        public static final double elevatorIdleSetpoint = 0;  //not sure if we need this//TODO: need to be changed 2025
        public static final double elevatorL1 = 70;//TODO: need to be changed 2025
        public static final double elevatorL2 = 70;//TODO: need to be changed 2025
        public static final double elevatorL3 = 70;//TODO: need to be changed 2025
        public static final double elevatorL4 = 70;//TODO: need to be changed 2025
        public static final double elevatorIntake = 60;//TODO: need to be changed 2025
        //FIXME: set actual port values and reversed for elevator encoder
        public static final int k_ENC_PORT = 2;//TODO: need to be changed 2025
        /*Elevator END */



        /*Wrist START */
        public static final int wristMotorID = 20;//TODO: need to be changed 2025
        public static final boolean wristMotorInverted = false;//TODO: need to be changed 2025

        //Wrist PID and Feedforward//TODO: need to be changed 2025

        /*Wrist END */



        /*Intake START*/
        public static final int coralMotorID = 15;//TODO: need to be changed 2025
        public static final int algaeMotorID = 16;//TODO: need to be changed 2025

        public static final boolean coralMotorInverted = false;//TODO: need to be changed 2025
        public static final boolean algaeMotorInverted = false;//TODO: need to be changed 2025
        
        public static final int coralLimitSwitchID = 1;//TODO: need to be changed 2025
        public static final int algaeLimitSwitchID = 2;//TODO: need to be changed 2025
        
        public static final int coralLimitSwitchPort = 1;//TODO: need to be changed 2025
        public static final int algaeLimitSwitchPort = 9;//TODO: need to be changed 2025

        public static final double intakeCoralSpeed = 0.5;//TODO: need to be changed 2025
        public static final double ejectCoralSpeed = .8;//TODO: need to be changed 2025
        public static final double intakeAlgaeSpeed = 0.5;//TODO: need to be changed 2025
        public static final double ejectAlgaeSpeed = .8;//TODO: need to be changed 2025
        

        /*Use these pid and feedforward stuff is we are tryign to RPM contorl */
        /*Coral intake pid optional if limit switch works*/
        public static final double coral_kP = 0.2;//TODO: need to be changed 2025
        public static final double coral_kI = 0;//TODO: need to be changed 2025
        public static final double coral_kD = 0;//TODO: need to be changed 2025
        
        /*Algea intake pid, optional if using limit switch */
        public static final double algea_kP = 0.2;//TODO: need to be changed 2025
        public static final double algea_kI = 0;//TODO: need to be changed 2025
        public static final double algea_kD = 0;//TODO: need to be changed 2025

        public static final double[] velocityPIDConstants = {0.00005,0,0};//shooter stuff from 2024, not sure if we need this//TODO: need to be changed 2025

        /*Feed forward variables, not sure if we need this*/
        public static final double coral_Ks = 0.00009;//TODO: need to be changed 2025
        public static final double coral_Kv = 0.000184;//TODO: need to be changed 2025
        
        public static final double algea_Ks = 0.00009;//TODO: need to be changed 2025
        public static final double algea_Kv = 0.000184;//TODO: need to be changed 2025
        
        public static final double intakeRPMSpeed = 1000;//TODO: need to be changed 2025

        /*rpm control stuff ends here */
        /*Intake END */
    }

    public static final class ShooterConstants{
        //FIXME: set motor IDs
        //Green
        public static final int shooterMotorAID = 16;
        //Orange
        public static final int shooterMotorBID = 17;

        /* Motor Inverts */
        public static final boolean motorAInverted = false;
        public static final boolean motorBInverted = true;

        /* Motor Neutral Modes */
        public static final NeutralModeValue MOTOR_A_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;//TODO: NEED CHANGE?
        public static final NeutralModeValue MOTOR_B_NEUTRAL_MODE_VALUE = NeutralModeValue.Coast;//TODO: NEED CHANGE?
        
        /* Shooter Current Limits */
        public static final boolean shooterAEnableCurrentLimit = true;
        public static final double shooterACurrentLimit = 25; //TODO: NEED CHANGE
        public static final int shooterACurrentThreshold = 25;//TODO: NEED CHANGE
        public static final double shooterACurrentThresholdTime = 0;

        public static final boolean shooterBEnableCurrentLimit = true;
        public static final double shooterBCurrentLimit = 25; //TODO: NEED CHANGE
        public static final int shooterBCurrentThreshold = 25;//TODO:NEED CHANGE
        public static final double shooterBCurrentThresholdTime = 0;


        public static final double idleOutput = .05;
        public static final double fireTime = 1;

        public static final double minShooterVelA = 2800;
        public static final double minShooterVelB = 2800;

        public static final double maxShooterVelA = 4400;
        public static final double maxShooterVelB = 4400;
        //FIXME: set break beam port
        // public static final int breakBeamPort = 0;

        //FIXME: set shooter velocity pid
        public static final double kPA = 0.000;
        public static final double kPB = 0.000;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double kFFkS = 0;
        public static final double kFFkVA = 0.00018;
        public static final double kFFkVB = 0.000175;
        public static final double kFFkA = 0;

        //FIXME: create lookup table
        public static final double[][] shooterLookupTable = {
            {2800,2800,.9779},
            {3000,3000,1.4986},
            {3200,3200,1.905},
            {3400,3400,2.8194},
            {3600,3600,3.302},
            {3900,3900,4.0132},
            {4400,4400,4.9784}
        };
        public static double[] getVelocitiesFromDistance(double distance) {
            double[] velocities = new double[2];

            if(distance < shooterLookupTable[0][2]) {
                velocities[0] = minShooterVelA;
                velocities[1] = minShooterVelB;
                return velocities;
            }
            if(distance > shooterLookupTable[shooterLookupTable.length-1][2]) {
                velocities[0] = maxShooterVelA;
                velocities[1] = maxShooterVelB;
                return velocities;
            }

            double[] smaller = new double[3];
            double[] larger = new double[3];

            for(int i = 0; i < shooterLookupTable.length-1; i++) {
                if(distance >= shooterLookupTable[i][2] && distance <= shooterLookupTable[i+1][2]) {
                    smaller = shooterLookupTable[i];
                    larger = shooterLookupTable[i+1];
                    break;
                }
            }
            //Y = Y1 + (X - X1) * ((Y2 - Y1)/(X2 - X1))
            velocities[0] = smaller[0] + (distance - smaller[2]) * ((larger[0]-smaller[0])/(larger[2]-smaller[2]));
            velocities[1] = smaller[0] + (distance - smaller[2]) * ((larger[1]-smaller[1])/(larger[2]-smaller[2]));
            return velocities;
        }
    }

    public static final class VisionConstants{

        public static final double fieldBorderMargin = 0.25;
        public static final double zMargin = 0.5;
        public static final double xyStdDevCoefficient = 0.02;
        public static final double thetaStdDevCoefficient = 0.04;
        public static final double ambiguityThreshold = 0.15;

        public static final Translation2d fieldSize = new Translation2d(16.54, 8.21);

        public static final String LIMELIGHT3_NAME_STRING = "limelight";
        public static final String LIMELIGHT2_NAME_STRING = "Limelight_2";


        public static final Pose2d SPEAKER_POSE2D_BLUE = new Pose2d(new Translation2d(-.0381, 5.547868), new Rotation2d(0));
        public static final Pose2d SPEAKER_POSE2D_RED = new Pose2d(new Translation2d(16.5793, 5.547868), new Rotation2d(180));
        public static final Pose2d AMP_POSE2D_RED = new Pose2d(new Translation2d(Units.inchesToMeters(580.77), Units.inchesToMeters(323-7.25)), new Rotation2d(270));
        public static final Pose2d AMP_POSE2D_BLUE = new Pose2d(new Translation2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323-7.25)), new Rotation2d(270));

        
        public static final Translation2d CENTER_OF_FIELD = new Translation2d(8.2706,4.105148);
        //FIXME: set limelight values
        public static final double limelightHeightInches = 0;
        public static final double limelightAngleDegrees = 0;

        public static HashMap<Integer, Double> tagHeights = new HashMap<Integer, Double>();

        // public static final Transform3d camToCenterRobotZero = new Transform3d(new Translation3d(-.254, -.254, 0.2159), new Rotation3d(0,Rotation2d.fromDegrees(50).getRadians(),0));//Cam mounted facing forward, half a meter forward of center, half a meter up from center. //TODO: need change
        // public static final Transform3d camToCenterRobotOne = new Transform3d(new Translation3d(.254, .254, 0.2159), new Rotation3d(0,Rotation2d.fromDegrees(-50).getRadians(),0));//Cam mounted facing forward, half a meter forward of center, half a meter up from center. //TODO: need change

        public static final Transform3d[] camerasToCenter = {
            new Transform3d(new Translation3d(.256032, -0.26035, 0.21209), new Rotation3d(0,Rotation2d.fromDegrees(-35).getRadians(),Rotation2d.fromDegrees(24.12).getRadians())),// Cam zero, left//TODO: need change
            new Transform3d(new Translation3d(.252222, 0.258318, 0.2159), new Rotation3d(0,Rotation2d.fromDegrees(-35).getRadians(),Rotation2d.fromDegrees(-16.90).getRadians()))//Cam one, right //TODO: need chagne
        };

        public static final double leftArduCamPitchOffsetRad = Rotation2d.fromDegrees(35).getRadians();
        public static final double rightArduCamPitchOffsetRad = Rotation2d.fromDegrees(35).getRadians();

        /**Trust value of the vision */
        public static final double visionStdDev = 0.5;

        public static void setTagHeights(){
            tagHeights.put(1, 48.125);
            tagHeights.put(2, 48.125);
            tagHeights.put(9, 48.125);
            tagHeights.put(10, 48.125);
            tagHeights.put(3, 53.875);
            tagHeights.put(4, 53.875);
            tagHeights.put(7, 53.875);
            tagHeights.put(8, 53.875);
            tagHeights.put(5, 53.125);
            tagHeights.put(6, 53.125);
            tagHeights.put(11, 47.5);
            tagHeights.put(12, 47.5);
            tagHeights.put(13, 47.5);
            tagHeights.put(14, 47.5);
            tagHeights.put(15, 47.5);
            tagHeights.put(16, 47.5);
        }

        public static final double[] aprilTagHeightInches = 
        {
            53.38,
            53.38,
            57.13,
            57.13,
            53.38,
            53.38,
            57.13,
            57.13,
            53.38,
            53.38,
            52.00,
            52.00,
            52.00,
            52.00,
            52.00,
            52.00
        };

        public static final double desiredDistanceToAprilTagY = 10; //DO NOT USE THIS BEFORE TUNE, DELTE AFTER TUNED TODO: CAD SPECS

        public static final double limelightMountAngleDegrees = 0; //TODO: CAD SPECS.

        public static final double heightOfCamAboveFloor = 2; //TODO: CAD SPECS
        // public static final double speakerTagID = ALLIANCE_COLOR.isPresent()
        //                                     ?
        //                                         ALLIANCE_COLOR.get() == DriverStation.Alliance.Red
        //                                         ?
        //                                             4d
        //                                         :
        //                                             7d
        //                                     :
        //                                         -1d;
                                                     
    }

    public static final class SwerveConstants {

        public static int swerveAlignUpdateSecond = 20;

        public static final int pigeonID = 1;

        public static final double translation_kP = 2.518;
        public static final double translation_kI = 0.6;
        public static final double translation_kD = 0.0;
        public static final double rotation_kP = 1.35;
        public static final double rotation_kI = 1.25;
        public static final double rotation_kD = 0.0;

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21); //TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(26); //TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double wheelRadius = chosenModule.wheelDiameter/2;

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
        
        /* Swerve module configs -- for pathplanner autobuilder (auto)
         * API: https://pathplanner.dev/api/java/com/pathplanner/lib/config/ModuleConfig.html
         */
        public static final DCMotor krackonX60 = new DCMotor(12, 7.09, 366, 2, 628.32, 4);//https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
        public static final ModuleConfig swerveModuleConfig = new ModuleConfig(wheelRadius,SwerveConstants.maxSpeed,1.0,krackonX60, Robot.ctreConfigs.swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit,4);
        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final boolean angleMotorInvert = chosenModule.driveMotorInvert == InvertedValue.CounterClockwise_Positive;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveMaxCurrent = 60;
        public static final double driveMaxCurrentTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.5;
        public static final double angleKI = 0;
        public static final double angleKD = 0.15;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; //TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; //TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;


        //Turning Pid Constants
        public static final double turnKP = 5;
        public static final double turnKD = 0;
        public static final double turnKI = 1.7;
        public static final double turnMaxVel = 400;
        public static final double turnMaxAccel = 800;
        public static final double turnTolerance = 1.75;
        public static final double turnIZone = .4;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 6380.0 / 60.0 * wheelCircumference * driveGearRatio;
        /** Radians per Second */
        public static final double maxAngularVelocity = maxSpeed / Math.hypot(trackWidth / 2.0, wheelBase / 2.0);

        /* Neutral Modes */
        public static final IdleMode angleNeutralMode = IdleMode.kBrake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

       /* Module Specific Constants */
        // Back Right Module 0
        public static final class Mod0 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 4;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-3.8671875);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        // Back Left Module 1
        public static final class Mod1 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-148.8867);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        //Front Right - Module 2
        public static final class Mod2 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(2.373046875);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        //Front left Module 3
        public static final class Mod3 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(114.697265625);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }

    public static final class AutoConstants { //FIXME: The below constants are used in the example auto, and must be tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 10;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double translation_kP = 5;
        public static final double translation_kI = 0.05;
        public static final double translation_kD = 0;
        public static final double rotation_kP = 0.45;
        public static final double rotation_kI = 0;
        public static final double rotation_kD = 0.05;
        public static final double rotationMaxAccel = 120;
        public static final double rotationMaxVel = 240;

        public static final double autoMaxVelocityMps = 5;
        public static final double autoMaxAcceleratMpsSq = 15;
        public static final double maxAngularVelocityRps = Rotation2d.fromDegrees(240).getRadians();
        public static final double maxAngularAcceleratRpsSq = Rotation2d.fromDegrees(480).getRadians();

        public static final double kPXController = 1.5;
        public static final double kPYController = 1.5;
        public static final double kPThetaController = 3;
    
        public static final PathConstraints pathConstraints = new PathConstraints(autoMaxVelocityMps, kMaxAccelerationMetersPerSecondSquared, maxAngularVelocityRps, maxAngularAcceleratRpsSq);

        // public static final Pose2d AMP_POSE2D = isRed ? new Pose2d(14.65, 7.63, new Rotation2d(Rotation2d.fromDegrees(90).getRadians())) : new Pose2d(1.9, 7.63, new Rotation2d(Rotation2d.fromDegrees(90).getRadians()));
        public static final Pose2d AMP_POSE2D = new Pose2d(AlienceColorCoordinateFlip.flip(2.0), 7.67, new Rotation2d(Rotation2d.fromDegrees(90).getRadians()));
        // public static final double maxXDistance = isRed ? 8.81 : 7.75;
        public static final double maxXDistance = isRed.equals("red") ? 8.6 : 8; // maximum x distance during auto so that it doesn't cross the middle of the field

        
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

        public static final Pose2d[] CENTERNOTE_POSE2DS =
        {
            new Pose2d(AlienceColorCoordinateFlip.flip(6.5), 7.5, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(180))),//Top one
            new Pose2d(AlienceColorCoordinateFlip.flip(6.5),5.7, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(180))),
            new Pose2d(AlienceColorCoordinateFlip.flip(6.5), 4.1, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(180))),
            new Pose2d(AlienceColorCoordinateFlip.flip(6.5), 2.45, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(180))),
            new Pose2d(AlienceColorCoordinateFlip.flip(6.5), 0.75, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(180)))
        };
        
        public static final Pose2d[] NEARNOTE_POSE2DS =
        {
            new Pose2d(AlienceColorCoordinateFlip.flip(2.15), 7, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(180))), //Top BLUE SIDE
            new Pose2d(AlienceColorCoordinateFlip.flip(2.15), 5.55, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(180))),
            new Pose2d(AlienceColorCoordinateFlip.flip(2.15), 4.1, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(180)))
        };

        public static final Pose2d[] MULTITARGETPOSES_FORINTAKECAMERA =
        {
            new Pose2d(6.00, 6.75, new Rotation2d(Rotation2d.fromDegrees(-170).getRadians())),
            new Pose2d(6.00, 4.00, new Rotation2d(Rotation2d.fromDegrees(-161.98).getRadians())),
            new Pose2d(6.00, 1.45, new Rotation2d(Rotation2d.fromDegrees(-170).getRadians()))
        };
        public static final Pose2d[] MULTITARGETPOSES_FORINTAKECAMERA_RED =
        {
            new Pose2d(AlienceColorCoordinateFlip.flip(6.00), 6.75, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(-170))),
            new Pose2d(AlienceColorCoordinateFlip.flip(6.00), 4.00, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(-161.98))),
            new Pose2d(AlienceColorCoordinateFlip.flip(6.00), 1.45, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(-170)))
        };

        /**These are specially for the bezier points, as their rotation 2d is the heading of the curve, not the robot base itself. */
        // public static final Pose2d[] BEZIER_CENTERNOTE_POSE2DS = 
        // {
        //     new Pose2d(AlienceColorCoordinateFlip.flip(7.6), 7.45, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(90))),//Top one
        //     new Pose2d(AlienceColorCoordinateFlip.flip(7.6),5.8, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(90))),
        //     new Pose2d(AlienceColorCoordinateFlip.flip(7.6), 4.1, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(90))),
        //     new Pose2d(AlienceColorCoordinateFlip.flip(7.6), 2.45, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(90))),
        //     new Pose2d(AlienceColorCoordinateFlip.flip(7.6), 0.75, new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(90)))
        // };
        /** Top shooting pose, top on pathplanner
         * @note that it is very close to the speaker, which means in order to get there you must need to travel through the three notes.
         */
        public static final Pose2d topShootingPose = new Pose2d(AlienceColorCoordinateFlip.flip(1.6), 6.25, new Rotation2d(Rotation2d.fromDegrees(AlienceColorCoordinateFlip.flipDegrees(-165)).getRadians()));

    
        public static final Pose2d openSideShootingPose = new Pose2d(AlienceColorCoordinateFlip.flip(1.85), 3.5, new Rotation2d(Rotation2d.fromDegrees(AlienceColorCoordinateFlip.flipDegrees(140)).getRadians()));
        
        public static final Pose2d middleShootingPose = new Pose2d(AlienceColorCoordinateFlip.flip(3.85), 5.50, new Rotation2d(Rotation2d.fromDegrees(AlienceColorCoordinateFlip.flipDegrees(180)).getRadians()));
        /**This value is increasement of currentcenternotepos, positive for it to go from top of the field in pathplanner, negative for it to go from the bottom to the top*/
        // public static final int centernoteIncrementVal = 1; //DO NOT CHANGE THIS VALUE (go from big to little in notePoseIDForAttempting instead)

        public static final double bufferVelocityForInBetweenPaths = 4;

        public static final double bufferVelocityForIntake = 2;

        public static final double bufferVelocityForShooting = 2;

        // public static double firstShootDelayInSeconds = 0.2;

        // public static int howManyNotesAreWeAttempting = 2;

        // public static int[] notePoseIDForAttempting = 
        // {
            // 0,
            // 1
        // };

        /**
         * Starting index for the pose that the robot will attempt
         * @IMPORTANT If go from BOTTOM to TOP, set this NO LOWER THAN MIN, if from TOP to BOTTOM, NO HIGHER THAN MAX
         */
        // public static int currentCenterNotePos = 0;//Starting index for the pose that the robot will attempt

        // public static final int centerNoteMax = 4; //from 0 to 4, 0 is top
        // public static final int centerNoteMin = 0;//from 0 to 4


    }

    public static final class ClimberConstants{
        //FIXME: set motor IDs
        public static final int climberDriverLeftID = 18;
        public static final int climberDriverRightID = 19;

        public static final boolean motorAInverted = false;
        public static final boolean motorBInverted = true;

        //FIXME: set setpoints
        public static final double topSetpoint = 0;
        public static final double bottomSetpoint = 0;
        public static final double climbedSetpoint = 0;

        //FIXME: set PID constants
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
    }

    public static final class blinkinConstants {
        public static final int PWMPort = 9;
    }

    public static final class WristConstants{
        public static final int WristMotorID = 10;

        public static final double kP = 0, kI = 0, kD = 0;
        public static final double kS = 0, kG = 0, kV = 0, kA = 0;
        public static final double allowedClosedLoopError = 0.5;
        public static final double maxAcceleration = 5, maxVelocity = 10;//Accelaration is in units of RPM per Second (RPM/s) & Maximum Velocity is in units of Revolutions per Minute (RPM)
        public static final double wristMaxDegrees = 60, wristMinDegrees = -60;
    }
}
