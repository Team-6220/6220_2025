package frc.robot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
// import com.ctre.phoenix6.signals.InvertedValue;

// import frc.robot.Constants.WristConstants;;

public final class CTREConfigs {
  // public TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();
  public TalonFXConfiguration swerveDriveFXConfig = new TalonFXConfiguration();
  public CANcoderConfiguration swerveCANcoderConfig = new CANcoderConfiguration();

  public CTREConfigs() {
    /** Swerve CANCoder Configuration */
    swerveCANcoderConfig.MagnetSensor.SensorDirection = Constants.SwerveConstants.cancoderInvert;

    /** Swerve Drive Motor Configuration */
    /* Motor Inverts and Neutral Mode */
    swerveDriveFXConfig.MotorOutput.Inverted = Constants.SwerveConstants.driveMotorInvert;
    swerveDriveFXConfig.MotorOutput.NeutralMode = Constants.SwerveConstants.driveNeutralMode;

    /* Gear Ratio Config */
    swerveDriveFXConfig.Feedback.SensorToMechanismRatio = Constants.SwerveConstants.driveGearRatio;

    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimitEnable =
        Constants.SwerveConstants.driveEnableCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLimit =
        Constants.SwerveConstants.driveMaxCurrent;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerLimit =
        Constants.SwerveConstants.driveCurrentLimit;
    swerveDriveFXConfig.CurrentLimits.SupplyCurrentLowerTime =
        Constants.SwerveConstants.driveMaxCurrentTime;

    /* PID Config */
    swerveDriveFXConfig.Slot0.kP = Constants.SwerveConstants.driveKP;
    swerveDriveFXConfig.Slot0.kI = Constants.SwerveConstants.driveKI;
    swerveDriveFXConfig.Slot0.kD = Constants.SwerveConstants.driveKD;

    /* Open and Closed Loop Ramping */
    swerveDriveFXConfig.OpenLoopRamps.DutyCycleOpenLoopRampPeriod =
        Constants.SwerveConstants.openLoopRamp;
    swerveDriveFXConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod =
        Constants.SwerveConstants.openLoopRamp;

    swerveDriveFXConfig.ClosedLoopRamps.DutyCycleClosedLoopRampPeriod =
        Constants.SwerveConstants.closedLoopRamp;
    swerveDriveFXConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod =
        Constants.SwerveConstants.closedLoopRamp;
  }
}
