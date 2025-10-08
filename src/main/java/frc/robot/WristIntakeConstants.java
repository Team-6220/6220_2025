package frc.robot;

import com.ctre.phoenix6.signals.NeutralModeValue;

public final class WristIntakeConstants {
  public static final NeutralModeValue INTAKENEU_NEUTRAL_MODE = NeutralModeValue.Brake;
  public static final int wristintakeMotorID = 18;
  public static final double ejectSpeed = .1;
  public static final double intakeSpeed = .1;

  public static final boolean enableCurrentLimit = true;
  public static final double maxCurrent = 20;
  public static final double currentLimit = 5;
  public static final double maxCurrentTime = 1;
}
