package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class FrontIntakeConstants {
  // TODO: TUNE ALL THESE VALUES
  public static final int rightMotorID = 15; // nonclimber
  public static final IdleMode rightMotorIdleMode = IdleMode.kBrake;
  public static final boolean rightMotorInvert = true;

  public static final int leftMotorID = 16; // climber
  public static final IdleMode leftMotorIdleMode = IdleMode.kBrake;
  public static final boolean leftMotorInvert = false;

  public static final boolean enableCurrentLimit = true;
  public static final double maxCurrent = 25;
  public static final double currentLimit = 20;
  public static final double maxCurrentTime = 2;

  public static final boolean enableStatorCurrentLimit = true;
  public static final double maxStatorCurrent = 40;

  public static final int stallLimit = 25; // amps
  public static final int freeLimit = 25; // amps

  public static final int frontMotorID = 19;

  public static final IdleMode frontMotorIdleMode = IdleMode.kCoast;
  public static final boolean frontMotorInvert = false;
  public static final double frontIntakeKp = 0.09; // .4;
  public static final double frontIntakeKi = 0.01;
  public static final double frontIntakeKd = 0.0;
  public static final double frontIntakeKa = 0.0;
  public static final double frontIntakeKg = 1.2; // .01;
  public static final double frontIntakeKv = 0.01;
  public static final double frontIntakeKs = 0;
  public static final double frontIntakeIZone = 5;
  public static final double frontIntakeTolerance = 0.5;
  public static final double frontIntakeMaxVel = 200;
  public static final double frontIntakeMaxAccel =
      800; // note : everytime increase max accel & velocity decrease kd

  /** volts, used for intake and only intake */
  public static final double wheelSpeed = 5;

  public static final double idleSpinVoltage = 2.5;

  public static final double maxDegrees = 120;
  public static final double minDegrees = -10;

  public static final double intakeAlgeaSetpoint = 65;
  public static final double intakeCoralSetpoint = -5;
  public static final double idleSetpoint = 106;
}
