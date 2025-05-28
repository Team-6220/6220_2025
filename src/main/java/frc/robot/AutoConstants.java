package frc.robot;

import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.lib.util.AlienceColorCoordinateFlip;

public final class AutoConstants {
  // FIXME: The below constants are used in the example auto, and must be
  // tuned to specific robot
  public static final double kMaxSpeedMetersPerSecond = 10;
  public static final double kMaxAccelerationMetersPerSecondSquared = 3;
  public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
  public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

  public static final double translation_kP = 2.25;
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

  public static final PathConstraints pathConstraints =
      new PathConstraints(
          autoMaxVelocityMps,
          kMaxAccelerationMetersPerSecondSquared,
          maxAngularVelocityRps,
          maxAngularAcceleratRpsSq);

  /* Constraint for the motion profilied robot angle controller */
  public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
          kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);

  public static final Pose2d[] MULTITARGETPOSES_FORINTAKECAMERA = {
    new Pose2d(6.00, 6.75, new Rotation2d(Rotation2d.fromDegrees(-170).getRadians())),
    new Pose2d(6.00, 4.00, new Rotation2d(Rotation2d.fromDegrees(-161.98).getRadians())),
    new Pose2d(6.00, 1.45, new Rotation2d(Rotation2d.fromDegrees(-170).getRadians()))
  };
  public static final Pose2d[] MULTITARGETPOSES_FORINTAKECAMERA_RED = {
    new Pose2d(
        AlienceColorCoordinateFlip.flip(6.00),
        6.75,
        new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(-170))),
    new Pose2d(
        AlienceColorCoordinateFlip.flip(6.00),
        4.00,
        new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(-161.98))),
    new Pose2d(
        AlienceColorCoordinateFlip.flip(6.00),
        1.45,
        new Rotation2d(AlienceColorCoordinateFlip.flipDegrees(-170)))
  };

  /**
   * This value is increasement of currentcenternotepos, positive for it to go from top of the field
   * in pathplanner, negative for it to go from the bottom to the top
   */
  // public static final int centernoteIncrementVal = 1; //DO NOT CHANGE THIS VALUE (go from big
  // to little in notePoseIDForAttempting instead)

  public static final double bufferVelocityForInBetweenPaths = 4;

  public static final double bufferVelocityForIntake = 2;

  public static final double bufferVelocityForShooting = 2;

  public static final Pose2d[] waypointPosesBlue = {
    new Pose2d(2.4, 4.1, new Rotation2d(0)), // first (closest to the drivers) reef pose
    new Pose2d(3.37, 2.3, new Rotation2d(Math.PI / 3)), // second reef pose rotating ccw
    new Pose2d(5.55, 2.28, new Rotation2d(2 * Math.PI / 3)), // third
    new Pose2d(6.59, 3.95, new Rotation2d(Math.PI)), // fourth
    new Pose2d(5.6, 5.8, new Rotation2d(-2 * Math.PI / 3)), // fifth
    new Pose2d(3.55, 5.86, new Rotation2d(-1 * Math.PI / 3)), // sixth
    new Pose2d(1.62, 1.37, new Rotation2d(-.7 * Math.PI)), // coral station to the right of drivers
    new Pose2d(1.46, 6.72, new Rotation2d(.7 * Math.PI)), // coral station to the left of drivers
    new Pose2d(11.53, 7.1, new Rotation2d(Math.PI / 2)) // processor
  };

  public static final Pose2d[] startPosesBlue = {
    new Pose2d(7.58, 7.25, new Rotation2d(0)), // outermost start pos for blue
    new Pose2d(7.58, 6.15, new Rotation2d(0)),
    new Pose2d(7.58, 5.06, new Rotation2d(0)),
    new Pose2d(7.58, 3, new Rotation2d(0)),
    new Pose2d(7.58, 1.9, new Rotation2d(0)),
    new Pose2d(7.58, .8, new Rotation2d(0))
  };

  // public static double firstShootDelayInSeconds = 0.2;

  // public static int howManyNotesAreWeAttempting = 2;

  // public static int[] notePoseIDForAttempting =
  // {
  // 0,
  // 1
  // };

  /**
   * Starting index for the pose that the robot will attempt @IMPORTANT If go from BOTTOM to TOP,
   * set this NO LOWER THAN MIN, if from TOP to BOTTOM, NO HIGHER THAN MAX
   */
  // public static int currentCenterNotePos = 0;//Starting index for the pose that the robot will
  // attempt

  // public static final int centerNoteMax = 4; //from 0 to 4, 0 is top
  // public static final int centerNoteMin = 0;//from 0 to 4

}
