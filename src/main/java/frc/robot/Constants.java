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
import com.fasterxml.jackson.annotation.JsonTypeInfo.Id;
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
    public final TunableNumber autoMaxAcceleratMpsSq = new TunableNumber("autoMaxAcceleratMpsSq",
            AutoConstants.autoMaxAcceleratMpsSq);
    public final TunableNumber maxAngularVelocityRps = new TunableNumber("maxAngularVelocityRps",
            AutoConstants.maxAngularVelocityRps);
    public final TunableNumber maxAngularAcceleratRpsSq = new TunableNumber("maxAngularAcceleratRpsSq",
            AutoConstants.maxAngularAcceleratRpsSq);

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

        /*
         * Start from zero after dead band.
         * Eg. if your deadband is .05
         * if you don't have this function the minimum input would be .05
         * if you use this function the input would be 0 when the joystick reading is at
         * .05
         */
        public static double modifyMoveAxis(double value) {
            // Deadband
            if (Math.abs(value) < kDeadband) {
                return 0;
            }

            // Change the axis
            double b = .1;
            double a = .5;
            double x = value;
            if (value >= 0) {
                return b + (1 - b) * (a * Math.pow(x, 3) + (1 - a) * x);
            } else {
                return -b + (1 - b) * (a * Math.pow(x, 3) + (1 - a) * x);
            }
            // value = Math.copySign(value * value, value);

            // return value;
        }

        public static double modifyRotAxis(double value) {
            // Deadband
            if (Math.abs(value) < kDeadband) {
                return 0;
            }

            // Change the axis
            double b = .05;
            double a = .2;
            if (value >= 0) {
                return b + (1 - b) * (a * Math.pow(value, 3) + (1 - a) * value);
            } else {
                return -b + (1 - b) * (a * Math.pow(value, 3) + (1 - a) * value);
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

            int invert = (Constants.isRed.equals("red")) ? -1 : 1;

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

        public static final COTSTalonFXSwerveConstants chosenModule = COTSTalonFXSwerveConstants.SDS.MK4i
                .KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21); // TODO: This must be tuned to specific robot
        public static final double wheelBase = Units.inchesToMeters(26); // TODO: This must be tuned to specific robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;
        public static final double wheelRadius = chosenModule.wheelDiameter / 2;

<<<<<<< HEAD
        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));
=======
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
>>>>>>> implement-elevator

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
        
<<<<<<< HEAD
        public static final double driveCurrentLimit = 35;
=======
        public static final int driveCurrentLimit = 35;
>>>>>>> implement-elevator
        public static final double driveMaxCurrent = 60;
        public static final double driveMaxCurrentTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;
        
<<<<<<< HEAD
        /*
         * Swerve module configs -- for pathplanner autobuilder (auto)
         * API:
         * https://pathplanner.dev/api/java/com/pathplanner/lib/config/ModuleConfig.html
         */
        public static final DCMotor krackonX60 = new DCMotor(12, 7.09, 366, 2, 628.32, 4);// https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
        public static final ModuleConfig swerveModuleConfig = new ModuleConfig(wheelRadius, SwerveConstants.maxSpeed,
                1.0, krackonX60,driveCurrentLimit , 4);
        
        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
=======
        /* Swerve module configs -- for pathplanner autobuilder (auto)
         * API: https://pathplanner.dev/api/java/com/pathplanner/lib/config/ModuleConfig.html
         */
        public static final DCMotor krackonX60 = new DCMotor(12, 7.09, 366, 2, 628.32, 4);//https://docs.wcproducts.com/kraken-x60/kraken-x60-motor/overview-and-features/motor-performance
        public static final ModuleConfig swerveModuleConfig = new ModuleConfig(wheelRadius,SwerveConstants.maxSpeed,1.0,krackonX60, driveMaxCurrent,4);
        
        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
        * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
>>>>>>> implement-elevator
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;
        
        /* Angle Motor PID Values */
        public static final double angleKP = 0.5;
        public static final double angleKI = 0;
        public static final double angleKD = 0.15;
        
        /* Drive Motor PID Values */
        public static final double driveKP = 0.12; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        // Turning Pid Constants
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
        public static final class Mod0 { // FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 10;
            public static final int canCoderID = 4;
<<<<<<< HEAD
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-3.8671875);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
=======
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-59.35);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
>>>>>>> implement-elevator
        }

        // Back Left Module 1
        public static final class Mod1 { // FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 12;
            public static final int canCoderID = 2;
<<<<<<< HEAD
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-148.8867);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
=======
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-94);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
>>>>>>> implement-elevator
        }

        // Front Right - Module 2
        public static final class Mod2 { // FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 1;
<<<<<<< HEAD
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(2.373046875);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
=======
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(23.46);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
>>>>>>> implement-elevator
        }

        // Front left Module 3
        public static final class Mod3 { // FIXME: This must be tuned to specific robot
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 3;
<<<<<<< HEAD
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(114.697265625);
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
                    canCoderID, angleOffset);
=======
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(91.31);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
>>>>>>> implement-elevator
        }
    }

    public static final class AutoConstants { // FIXME: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
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

        public static final PathConstraints pathConstraints = new PathConstraints(autoMaxVelocityMps,
                kMaxAccelerationMetersPerSecondSquared, maxAngularVelocityRps, maxAngularAcceleratRpsSq);

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

    public static final class blinkinConstants {
        public static final int PWMPort = 9;
    }
    public static final class ElevatorConstants{
        //TODO: TUNE ALL THESE VALUES
        public static final int rightMotorID = 13;
        public static final IdleMode rightMotorIdleMode = IdleMode.kCoast;
        public static final boolean rightMotorInvert = false;

        public static final int leftMotorID = 14;
        public static final IdleMode leftMotorIdleMode = IdleMode.kCoast;
        public static final boolean leftMotorInvert = true;

        public static final double elevatorEncoderOffset = 0;//TODO: SET THIS
        public static final int elevatorEncoderID = 1;

        public static final double elevatorKp = 0.15;
        public static final double elevatorKi = 0.0;
        public static final double elevatorKd = 0.0;
        public static final double elevatorKg = 0.3;
        public static final double elevatorKv = 0.02;
        public static final double elevatorKa = 0.01;
        public static final double elevatorIZone = 3.0;
        public static final double elevatorTolerance = 1.5;
        public static final double elevatorMaxVel = 0.5
        ;
        public static final double elevatorMaxAccel = 0.5;

        public static final double L2HeightRaw = 10.0;//TODO: CHANGE THESE
        public static final double L3HeightRaw = 15.0;//TODO: CHANGE THESE
        public static final double L4HeightRaw = 20.0;//TODO: CHANGE THESE

        public static final double lowerEncoderExtreme = 0.0;
        public static final double upperEncoderExtreme = 55.0;
    }
    public static final class FrontIntakeConstants{
        //TODO: TUNE ALL THESE VALUES
        public static final int rightMotorID = 15; //nonclimber
        public static final IdleMode rightMotorIdleMode = IdleMode.kBrake;
        public static final boolean rightMotorInvert = true;

        public static final int leftMotorID = 16; //climber
        public static final IdleMode leftMotorIdleMode = IdleMode.kBrake;
        public static final boolean leftMotorInvert = false;
        public static final int frontMotorID = 19; 
        public static final IdleMode frontMotorIdleMode = IdleMode.kBrake;
        public static final boolean frontMotorInvert = false;
        public static final double frontIntakeKp = 0.4;//.4;
        public static final double frontIntakeKi = 0;
        public static final double frontIntakeKd = 0;
        public static final double frontIntakeKg = 0.01;//.01;
        public static final double frontIntakeKv = 0;
        public static final double frontIntakeKs = 0;
        public static final double frontIntakeIZone = 0;
        public static final double frontIntakeTolerance = 1.5;
        public static final double frontIntakeMaxVel = 5;
        public static final double frontIntakeMaxAccel = 10;
        public static final double wheelSpeed = 0.25; //0-1
    }
}