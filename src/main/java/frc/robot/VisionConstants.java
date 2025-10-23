package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public final class VisionConstants {

  public static final double fieldBorderMargin = 0.25;
  public static final double zMargin = 0.5;
  public static final double xyStdDevCoefficient = 0.02;
  public static final double thetaStdDevCoefficient = 0.04;
  public static final double ambiguityThreshold = 0.15;

  public static final Translation2d fieldSize = new Translation2d(16.54, 8.21);

  public static final String[] cameraNames = {
    "Bottom_Right_Cam", // Top right USB
    "Right_Ardu_Cam" // Bottom right usb
  };
  public static final Pose2d SPEAKER_POSE2D_BLUE =
      new Pose2d(new Translation2d(-.0381, 5.547868), new Rotation2d(0));
  public static final Pose2d SPEAKER_POSE2D_RED =
      new Pose2d(new Translation2d(16.5793, 5.547868), new Rotation2d(180));
  public static final Pose2d AMP_POSE2D_RED =
      new Pose2d(
          new Translation2d(Units.inchesToMeters(580.77), Units.inchesToMeters(323 - 7.25)),
          new Rotation2d(270));
  public static final Pose2d AMP_POSE2D_BLUE =
      new Pose2d(
          new Translation2d(Units.inchesToMeters(72.5), Units.inchesToMeters(323 - 7.25)),
          new Rotation2d(270));

  public static final Translation2d CENTER_OF_FIELD = new Translation2d(8.2706, 4.105148);
  // FIXME: set limelight values
  public static final double limelightHeightInches = 0;
  public static final double limelightAngleDegrees = 0;

  // public static final Transform3d camToCenterRobotZero = new Transform3d(new
  // Translation3d(-.254, -.254, 0.2159), new
  // Rotation3d(0,Rotation2d.fromDegrees(50).getRadians(),0));//Cam mounted facing forward, half a
  // meter forward of center, half a meter up from center. //TODO: need change
  // public static final Transform3d camToCenterRobotOne = new Transform3d(new Translation3d(.254,
  // .254, 0.2159), new Rotation3d(0,Rotation2d.fromDegrees(-50).getRadians(),0));//Cam mounted
  // facing forward, half a meter forward of center, half a meter up from center. //TODO: need
  // change

//   public static final Transform3d[] camerasToCenter = {
//     new Transform3d(
//         new Translation3d(.256032, -0.26035, 0.21209),
//         new Rotation3d(
//             0,
//             Rotation2d.fromDegrees(-35).getRadians(),
//             Rotation2d.fromDegrees(24.12).getRadians())), // Cam zero, left//TODO: need change
//     new Transform3d(
//         new Translation3d(.252222, 0.258318, 0.2159),
//         new Rotation3d(
//             0,
//             Rotation2d.fromDegrees(-35).getRadians(),
//             Rotation2d.fromDegrees(-16.90).getRadians())) // Cam one, right //TODO: need chagne
//   };

  public static final double[] cameraYawDegrees = 
  {
    28,
    0
  };

  public static final Transform3d[] robotCenterToCamera = {
    new Transform3d(
        new Translation3d(0.3, 0.2, 0.2),
        new Rotation3d(0, 0, Math.toRadians(40))),
    //TODO: as of 6/11/2025, the second camera's pose has not been set (just a rough estimation) because we can not have access to the robot
    new Transform3d(
        new Translation3d(0.0, 0.2, 0.5),  
        new Rotation3d(0, 0, Math.toRadians(40))
    )
};

  public static final double leftArduCamPitchOffsetRad = Rotation2d.fromDegrees(35).getRadians();
  public static final double rightArduCamPitchOffsetRad = Rotation2d.fromDegrees(35).getRadians();

  /** Trust value of the vision */
  public static final double visionStdDev = 0.5;

  // Height in inches for all April Tags in order from 1 to 22
  public static final double[] aprilTagHeightInches = {
    55.25, 55.25, 47.88, 70.73, 70.73, 8.75, 8.75, 8.75, 8.75, 8.75, 8.75, 55.25, 55.25, 47.88,
    70.73, 70.73, 8.75, 8.75, 8.75, 8.75, 8.75, 8.75
  };

  public static final double[] aprilTagCoordsX = {
    657.37, 657.37, 455.15, 365.20, 365.20, 530.49, 546.87, 530.49, 497.77, 481.39, 497.77, 33.51,
    33.51, 325.68, 325.68, 235.73, 160.39, 144.00, 160.39, 193.10, 209.49, 193.10
  };

  public static final double[] aprilTagCoordsY = {
    25.80, 291.20, 317.15, 241.64, 75.39, 130.17, 158.50, 186.83, 186.83, 158.50, 130.17, 25.80,
    291.20, 241.64, 75.39, -0.15, 130.17, 158.50, 186.83, 186.83, 158.50, 130.17
  };

    public static final double[] aprilTagYaw = {
      126.0, 234.0, 270.0, 0.0, 0.0, 300.0, 0.0, 60.0, 120.0, 180.0, 240.0, 54.0, 306.0, 180.0,
      180.0, 90.0, 240.0, 180.0, 120.0, 60.0, 0.0, 300.0
    };

    public static final AprilTag[] apriltags2025 = {
      new AprilTag(
          1, new Pose3d(16.687292, 0.628142, 1.4859, new Rotation3d(0.0, 0.0, 0.8910065241883678))),
      new AprilTag(
          2,
          new Pose3d(
              16.687292, 7.414259999999999, 1.4859, new Rotation3d(0.0, 0.0, 0.8910065241883679))),
      new AprilTag(
          3,
          new Pose3d(
              11.49096, 8.031733999999998, 1.30175, new Rotation3d(0.0, 0.0, 0.7071067811865476))),
      new AprilTag(
          4,
          new Pose3d(
              9.276079999999999,
              6.132575999999999,
              1.8679160000000001,
              new Rotation3d(0.0, 0.25881904510252074, 0.0))),
      new AprilTag(
          5,
          new Pose3d(
              9.276079999999999,
              1.9098259999999998,
              1.8679160000000001,
              new Rotation3d(0.0, 0.25881904510252074, 0.0))),
      new AprilTag(
          6,
          new Pose3d(
              13.474446,
              3.3012379999999997,
              0.308102,
              new Rotation3d(0.0, 0.0, 0.49999999999999994))),
      new AprilTag(
          7, new Pose3d(13.890498, 4.0208200000000005, 0.308102, new Rotation3d(0.0, 0.0, 0.0))),
      new AprilTag(
          8,
          new Pose3d(13.474446, 4.740402, 0.308102, new Rotation3d(0.0, 0.0, 0.49999999999999994))),
      new AprilTag(
          9,
          new Pose3d(12.643358, 4.740402, 0.308102, new Rotation3d(0.0, 0.0, 0.8660254037844386))),
      new AprilTag(
          10,
          new Pose3d(
              12.227305999999999, 4.0208200000000005, 0.308102, new Rotation3d(0.0, 0.0, 1.0))),
      new AprilTag(
          11,
          new Pose3d(
              12.643358,
              3.3012379999999997,
              0.308102,
              new Rotation3d(0.0, 0.0, 0.8660254037844387))),
      new AprilTag(
          12,
          new Pose3d(
              0.8613139999999999, 0.628142, 1.4859, new Rotation3d(0.0, 0.0, 0.45399049973954675))),
      new AprilTag(
          13,
          new Pose3d(
              0.8613139999999999,
              7.414259999999999,
              1.4859,
              new Rotation3d(0.0, 0.0, 0.45399049973954686))),
      new AprilTag(
          14,
          new Pose3d(
              8.272272,
              6.132575999999999,
              1.8679160000000001,
              new Rotation3d(-0.25881904510252074, 1.5848095757158825e-17, 0.9659258262890683))),
      new AprilTag(
          15,
          new Pose3d(
              8.272272,
              1.9098259999999998,
              1.8679160000000001,
              new Rotation3d(-0.25881904510252074, 1.5848095757158825e-17, 0.9659258262890683))),
      new AprilTag(
          16,
          new Pose3d(
              6.057646,
              0.010667999999999999,
              1.30175,
              new Rotation3d(0.0, 0.0, 0.7071067811865476))),
      new AprilTag(
          17,
          new Pose3d(
              4.073905999999999,
              3.3012379999999997,
              0.308102,
              new Rotation3d(0.0, 0.0, 0.8660254037844387))),
      new AprilTag(
          18, new Pose3d(3.6576, 4.0208200000000005, 0.308102, new Rotation3d(0.0, 0.0, 1.0))),
      new AprilTag(
          19,
          new Pose3d(
              4.073905999999999, 4.740402, 0.308102, new Rotation3d(0.0, 0.0, 0.8660254037844386))),
      new AprilTag(
          20,
          new Pose3d(
              4.904739999999999,
              4.740402,
              0.308102,
              new Rotation3d(0.0, 0.0, 0.49999999999999994))),
      new AprilTag(
          21, new Pose3d(5.321046, 4.0208200000000005, 0.308102, new Rotation3d(0.0, 0.0, 0.0))),
      new AprilTag(
          22,
          new Pose3d(
              4.904739999999999,
              3.3012379999999997,
              0.308102,
              new Rotation3d(0.0, 0.0, 0.49999999999999994)))
    };

    // creates a hash map of the X Y and Height in that order for april tags
    //NOTE: HASHMAP INDEX STARTS AT 1, NOT ZERO
    public static HashMap<Integer, ArrayList<Double>> aprilTagXYHeightYaw =
        new HashMap<Integer, ArrayList<Double>>();

  public static HashMap<Integer, Double[]> cameraSpecs = new HashMap<Integer, Double[]>();

    public static void setTagXYHeightAngle() {
      for (int i = 1; i <= aprilTagYaw.length; i++) {
        aprilTagXYHeightYaw.put(i, new ArrayList<Double>());
        aprilTagXYHeightYaw.get(i).add(aprilTagCoordsX[i - 1]);
        aprilTagXYHeightYaw.get(i).add(aprilTagCoordsY[i - 1]);
        aprilTagXYHeightYaw.get(i).add(aprilTagHeightInches[i - 1]);
        aprilTagXYHeightYaw.get(i).add(aprilTagYaw[i - 1]);
      }
      System.out.print(aprilTagXYHeightYaw);
      for (int x = 0; x < 3; x++) {
        cameraSpecs.put(x, new Double[2]);
        cameraSpecs.get(x)[0] = cameraHeight[x];
        cameraSpecs.get(x)[1] = cameraAngles[x];
      }
    }

  //                                     bottom right     top right       top left
  public static double[] cameraHeight = {29.5 + 1.724, 35.707 + 1.724, 35.707 + 1.724};

  /** Degrees */
  public static double[] cameraAngles = {210.0, 135.0, 45.0};

  public static final double centerCoralStationVisionX = .72387872;
  public static final double centerCoralStationVisionY = .053628;

  public static final double leftReefX = .552627000515524;
  public static final double leftReefY = 0.40943570030106996;

  public static final double rightReefX = 0.4737554651243565;
  public static final double rightReefY = 0.7163253241182086;

  // aprilTagXYHeightAngle.put(1, new Double[]{55.25, 657.37, 25.80, 126.0});

  public static final double heightOfCamAboveFloor = 2; // TODO: CAD SPECS
  public static final double speakerTagID =
      Constants.ALLIANCE_COLOR.isPresent()
          ? Constants.ALLIANCE_COLOR.get() == DriverStation.Alliance.Red ? 4d : 7d
          : -1d;
}
