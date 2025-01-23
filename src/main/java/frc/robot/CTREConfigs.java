// package frc.robot;

// import com.ctre.phoenix6.configs.CANcoderConfiguration;
// import com.ctre.phoenix6.configs.TalonFXConfiguration;
// // import com.ctre.phoenix6.signals.InvertedValue;

// // import frc.robot.Constants.WristConstants;;

// public final class CTREConfigs {
//     // public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
//     public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
//     public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

//     public CTREConfigs(){
//         /** Swerve CANCoder Configuration */
//         swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.cancoderInvert;

//         /** Swerve Angle Motor Configurations */
//         /* Motor Inverts and Neutral Mode */
//         //swerveAngleFXConfig.MotorOutput.Inverted = Constants.Swerve.angleMotorInvert;
//         // swerveAngleFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.angleNeutralMode;

//         /* Gear Ratio and Wrapping Config */
//         // swerveAngleFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.angleGearRatio;
//         // swerveAngleFXConfig.ClosedLoopGeneral.ContinuousWrap = true;
        
//         /* Current Limiting */
//         // swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.angleEnableCurrentLimit;
//         // swerveAngleFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.angleCurrentLimit;
//         // swerveAngleFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.SwerveConstants.angleCurrentThreshold;
//         // swerveAngleFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.SwerveConstants.angleCurrentThresholdTime;

//         /* PID Config */
//         // swerveAngleFXConfig.Slot0.kP = Constants.SwerveConstants.angleKP;
//         // swerveAngleFXConfig.Slot0.kI = Constants.SwerveConstants.angleKI;
//         // swerveAngleFXConfig.Slot0.kD = Constants.SwerveConstants.angleKD;

//         /** Swerve Drive Motor Configuration */
//         /* Motor Inverts and Neutral Mode */
//         swerveDriveFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.driveMotorInvert;
//         swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.driveNeutralMode;

//         /* Gear Ratio Config */
//         swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.driveGearRatio;

//         /* Current Limiting */
//         // swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
//         // swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.driveCurrentLimit;
//         // swerveDriveFXConfig.CurrentLimits.SupplyCurrentThreshold = Constants.SwerveConstants.driveCurrentThreshold;
//         // swerveDriveFXConfig.CurrentLimits.SupplyTimeThreshold = Constants.SwerveConstants.driveCurrentThresholdTime;

//         swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable = Constants.SwerveConstants.driveEnableCurrentLimit;
//         swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit = Constants.SwerveConstants.driveMaxCurrent;
//         swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit = Constants.SwerveConstants.driveCurrentLimit;
//         swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime = Constants.SwerveConstants.driveMaxCurrentTime;


//         /* PID Config */
//         swerveDriveFXConfig.Slot0.kP = Constants.SwerveConstants.driveKP;
//         swerveDriveFXConfig.Slot0.kI = Constants.SwerveConstants.driveKI;
//         swerveDriveFXConfig.Slot0.kD = Constants.SwerveConstants.driveKD;

//         /* Open and Closed Loop Ramping */
//         swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;
//         swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = Constants.SwerveConstants.openLoopRamp;

//         swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;
//         swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = Constants.SwerveConstants.closedLoopRamp;

//         /** Shooter Motor A, Green, Configuration */
//          /* Motor Inverts and Neutral Mode */
//         // wristConfig.MotorOutput.Inverted = WristConstants.wristMotorInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;
//         // wristConfig.MotorOutput.NeutralMode = WristConstants.WRISTMOTOR_NEUTRAL_MODE_VALUE;

//         // /* Gear Ratio Config */
//         // // wristConfig.Feedback.SensorToMechanismRatio = ;

//         // /* Current Limiting */
//         // wristConfig.CurrentLimits.SupplyCurrentLimitEnable = WristConstants.wristEnableCurrentLimit;
//         // wristConfig.CurrentLimits.SupplyCurrentLowerLimit = WristConstants.wristCurrentLimit;
//         // wristConfig.CurrentLimits.SupplyCurrentLimit = WristConstants.wristCurrentThreshold;
//         // wristConfig.CurrentLimits.SupplyCurrentLowerTime = WristConstants.wristCurrentThresholdTime;

//         // /* PID Config */
//         // wristConfig.Slot0.kP = ;
//         // wristConfig.Slot0.kI = ;
//         // wristConfig.Slot0.kD = ;

//         /* Open and Closed Loop Ramping */
//         // wristConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod = ;
//         // wristConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = ;

//         // wristConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod = ;
//         // wristConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = ;
//     }
// }