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

import com.ctre.phoenix.motorcontrol.NeutralMode;
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

    public static final Mass robotMass = Pound.of(112);
    public static final MomentOfInertia robotMOI = KilogramSquareMeters.of(4.563);

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;

        public static final double kDeadband = 0.085;

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

          public static double modifyVoltageAxis(double value) {
            // Deadband
            if (Math.abs(value) < kDeadband) {
                return 0;
            }
    
            // Change the axis
            double b = 0.1;
            double a = 0.5;
            double x = value;
    
            // Calculate modified value
            double modifiedValue;
            if (value >= 0) {
                modifiedValue = b + (1 - b) * (a * Math.pow(x, 3) + (1 - a) * x);
            } else {
                modifiedValue = -b + (1 - b) * (a * Math.pow(x, 3) + (1 - a) * x);
            }
    
            // Convert to voltage (assuming input ranges from 0 to 1 corresponds to 0 to 12 volts)
            double voltage = modifiedValue * 12;
    
            return voltage;
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
        
        public static final double driveCurrentLimit = 35;
        public static final double driveMaxCurrent = 60;
        public static final double driveMaxCurrentTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
       
        /* Swerve module configs -- for pathplanner autobuilder (auto)
         * API: https://pathplanner.dev/api/java/com/pathplanner/lib/config/ModuleConfig.html
         */
        public static final DCMotor krackonX60 = new DCMotor(12, 7.09, 366, 2, 628.32, 4);//https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
        public static final ModuleConfig swerveModuleConfig = new ModuleConfig(wheelRadius,SwerveConstants.maxSpeed,1.0,krackonX60, driveCurrentLimit,4);
        
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-58.447);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        // Back Left Module 1
        public static final class Mod1 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 2;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-94.5);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        //Front Right - Module 2
        public static final class Mod2 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 1;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(23.8);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        //Front left Module 3
        public static final class Mod3 { //FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 3;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(90.5);
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
        
        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
        

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

    public static final class blinkinConstants {
        public static final int PWMPort = 9;
    }

    public static final class WristConstants{
        public static final int WristMotorID = 17;
        public static final boolean motorInverted = false;
        public static final IdleMode wristIdleMode = IdleMode.kBrake;
        
        public static final boolean encoderInverted = false;
        public static final int stallLimit = 5;
        public static final int freeLimit = 20;
        
        public static final double kP = 0.00, kI = 0, kD = 0, izone = 2, tolerance = .5;
        public static final double kS = 0, kG = .15, kV = 1.75, kA = 0;
        public static final double allowedClosedLoopError = 0.5;
        public static final double maxAcceleration = 720, maxVelocity = 360;//Accelaration is in units of RPM per Second (RPM/s) & Maximum Velocity is in units of Revolutions per Minute (RPM)
        public static final double wristMaxDegrees = 87, wristMinDegrees = -144;

        public static final double L2 = -48.8018;
        public static final double L3 = -46.2412;
        public static final double L4 = -5;
        public static final double coralStation = 35.81813;
    }
    public static final class ElevatorConstants{
        //TODO: TUNE ALL THESE VALUES
        public static final int rightMotorID = 14;
        public static final IdleMode rightMotorIdleMode = IdleMode.kCoast;
        public static final boolean rightMotorInvert = true;

        public static final int leftMotorID = 13;
        public static final IdleMode leftMotorIdleMode = IdleMode.kCoast;
        public static final boolean leftMotorInvert = false;

        public static final int stallLimit = 20;
        public static final int freeLimit = 20;

        public static final double elevatorOffset = 0; //FOR COMP

        public static final double elevatorEncoderOffset = 0;//TODO: SET THIS
        public static final int elevatorEncoderID = 1;

        public static final double elevatorKp = 0.0;
        public static final double elevatorKi = 0.0;
        public static final double elevatorKd = 0.0;
        public static final double elevatorKg = 0.23;//Tune this first
        //carret in the middle, if it stil move up, lower it until it holds it in position
        //Then give a little kp to go to position
        //then increase max accel & vel to make it faster (after change unit of posiiotn to m, velocity is m/s)
        public static final double elevatorKv = 9.75;//Or 10.5, this was the old kv, something like that :) You got this Nathan //frc mechanism calculator, reca.lc --> linear machanism calculator -- put approximately
        public static final double elevatorKa = 15; //How fast they can go, max vel & accel puts a cap in case if it's too fast.
        //stall load -- how much weight it can handle at all
        public static final double elevatorKs = 0;//start with 0
        //if it's getting stuck to go down or up then increase ks by a little bit to fight friction
        //if rasiing ks might have to lower kg
        public static final double elevatorIZone = 0.1;
        public static final double elevatorTolerance = .005;
        public static final double elevatorMaxVel = 1.25;//Ok tune this a little higher/lower//meters per second
        public static final double elevatorMaxAccel = 4.2
        
        ;//I think you don't need to tune this one but you can//meters per second square

        
        //These values should be percents
        public static final double E_L2 = 0.485757;//This one should be good
        public static final double E_L3 = .929;//This one should be good
        public static final double E_L4 = 1.2126259;//Maybe final tune this?
        public static final double E_CoralStation = .210796;//Maybe final tune this?

        public static final double lowerEncoderExtreme = 0.0; 
        public static final double upperEncoderExtreme = 1.2;
        }
    public static final class FrontIntakeConstants{
        //TODO: TUNE ALL THESE VALUES
        public static final int rightMotorID = 15; //nonclimber
        public static final IdleMode rightMotorIdleMode = IdleMode.kBrake;
        public static final boolean rightMotorInvert = true;

        public static final int leftMotorID = 16; //climber
        public static final IdleMode leftMotorIdleMode = IdleMode.kBrake;
        public static final boolean leftMotorInvert = false;

        public static final boolean enableCurrentLimit = true;
        public static final double maxCurrent = 20;
        public static final double currentLimit = 15;
        public static final double maxCurrentTime = 1;

        public static final boolean enableStatorCurrentLimit = true;
        public static final double maxStatorCurrent = 40;

        public static final int stallLimit = 25; //amps
        public static final int freeLimit = 25; //amps

        public static final int frontMotorID = 19; 

        public static final IdleMode frontMotorIdleMode = IdleMode.kCoast;
        public static final boolean frontMotorInvert = false;
        public static final double frontIntakeKp = .05;//.4;
        public static final double frontIntakeKi = 0.05;
        public static final double frontIntakeKd = 0.0;
        public static final double frontIntakeKa = 0.0;
        public static final double frontIntakeKg = 0.87;//.01;
        public static final double frontIntakeKv = 0.35;
        public static final double frontIntakeKs = 0;
        public static final double frontIntakeIZone = 5;
        public static final double frontIntakeTolerance = 1.5;
        public static final double frontIntakeMaxVel = 200;
        public static final double frontIntakeMaxAccel = 800; //note : everytime increase max accel & velocity decrease kd
        /**volts, used for intake and only intake */
        public static final double wheelSpeed = 3;
        public static final double idleSpinVoltage = 1;

        public static final double maxDegrees = 120;
        public static final double minDegrees = -10;

        public static final double intakeAlgeaSetpoint = 65;
        public static final double intakeCoralSetpoint = -5;
        public static final double idleSetpoint = 106;

    }
    public static final class WristIntakeConstants {
        public static final NeutralModeValue INTAKENEU_NEUTRAL_MODE = NeutralModeValue.Brake;
        public static final int wristintakeMotorID = 18;
        public static final double ejectSpeed = .1;
        public static final double intakeSpeed = .1;

        public static final boolean enableCurrentLimit = true;
        public static final double maxCurrent = 20;
        public static final double currentLimit = 5;
        public static final double maxCurrentTime = 1;
    }

}
