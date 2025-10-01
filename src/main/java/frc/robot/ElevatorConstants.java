package frc.robot;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public final class ElevatorConstants {
  // TODO: TUNE ALL THESE VALUES
  public static final int rightMotorID = 14;
  public static final IdleMode rightMotorIdleMode = IdleMode.kCoast;
  public static final boolean rightMotorInvert = true;

  public static final int leftMotorID = 13;
  public static final IdleMode leftMotorIdleMode = IdleMode.kCoast;
  public static final boolean leftMotorInvert = false;

  public static final int stallLimit = 21;
  public static final int freeLimit = 21;

  public static final double elevatorOffset = 0; // FOR COMP

  public static final double elevatorEncoderOffset = 0; // TODO: SET THIS
  public static final int elevatorEncoderID = 1;

  public static final double elevatorKp = 0.075;
  public static final double elevatorKi = 0.5;
  public static final double elevatorKd = 0.0015;
  public static final double elevatorKg = 0.25; // Tune this first
  // carret in the middle, if it stil move up, lower it until it holds it in position
  // Then give a little kp to go to position
  // then increase max accel & vel to make it faster (after change unit of posiiotn to m, velocity
  // is m/s)
  public static final double elevatorKv = 8.8;
  // mechanism calculator, reca.lc --> linear machanism calculator -- put approximately
  public static final double elevatorKa =
      15; // How fast they can go, max vel & accel puts a cap in case if it's too fast.
  // stall load -- how much weight it can handle at all
  public static final double elevatorKs =
      0.22; // start with 0, how much to overcome static friction in the system
  // if it's getting stuck to go down or up then increase ks by a little bit to fight friction
  // if rasiing ks might have to lower kg
  public static final double elevatorIZone = 0.5;
  public static final double elevatorTolerance = .005;
  public static final double elevatorMaxVel =
      1; // Ok tune this a little higher/lower//meters per second
  public static final double elevatorMaxAccel =
      5; // I think you don't need to tune this one but you can//meters per second square

  // These values should be percents
  public static final double E_L2 = 0.485757; // This one should be good
  public static final double E_L3 = .837;
   // This one should be good
  public static final double E_L4 = 1.3; // Maybe final tune this?
  public static final double E_CoralStation = .195796; // Maybe final tune this?

  public static final double eleDeAlgeL2 = 0.090768744;
  public static final double eleDeAlgeL3 = 0.328161844;

  public static final double lowerEncoderExtreme = 0.0;
  public static final double upperEncoderExtreme = 1.3;
}
