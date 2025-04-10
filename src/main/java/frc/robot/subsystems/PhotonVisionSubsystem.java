package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  // "src\\main\\java\\frc\\lib\\vision\\2025-reefscape-andymark.json"
  // "src\main\deploy\vision\2025-reefscape-andymark.json"
  // public static Path path =
  // Filesystem.getDeployDirectory().toPath().resolve("2025-reefscape-andymark.json");
  private static PhotonCamera[] cameras;
  private static String[] cameraNames;
  private final long[] lastHeartbeats;

  public static Field2d theFieldCam0 = new Field2d(), theFieldCam1 = new Field2d();

  private PhotonTrackedTarget noErrorHopefully;

  private static PhotonVisionSubsystem INSTANCE = null;

  private static String tableKey = "Vision_";

  private HashMap<Integer, List<PhotonPipelineResult>> results;
  private HashMap<Integer, List<PhotonTrackedTarget>> bestTarget;

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem(String[] cameraNames) {
    cameras = new PhotonCamera[cameraNames.length];
    lastHeartbeats = new long[cameraNames.length];
    this.cameraNames = cameraNames;

    VisionConstants.setTagXYHeightAngle();

    initPhoton();

    results = new HashMap<Integer, List<PhotonPipelineResult>>();
    for (int i = 0; i < cameras.length; i++) {
      results.put(i, null);
    }

    bestTarget = new HashMap<Integer, List<PhotonTrackedTarget>>();
    for (int i = 0; i < cameras.length; i++) {
      bestTarget.put(i, null);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void initPhoton() {
    for (int i = 0; i < cameraNames.length; i++) {
      if (isCameraConnected(cameraNames[i])) {
        cameras[i] = new PhotonCamera(cameraNames[i]);
        System.out.println("Photon camera initialized: " + cameraNames[i]);
        cameras[i].setPipelineIndex(0);
        NetworkTable camTable =
            NetworkTableInstance.getDefault().getTable("photonvision/" + cameras[i].getName());
        NetworkTableEntry heartbeatEntry = camTable.getEntry("heartbeat");
        lastHeartbeats[i] = (long) heartbeatEntry.getDouble(-1);
      } else {
        cameras[i] = null;
        System.out.println("Photon camera not found: " + cameraNames[i]);
        lastHeartbeats[i] = -1;
      }
    }
  }

  public void updatePhoton() {
    for (int i = 0; i < cameras.length; i++) {
      if (cameras[i] == null) {
        results.put(i, null);
        bestTarget.put(i, new ArrayList<>());
        System.err.println(cameraNames[i] + " isNull");
        continue;
      }

      NetworkTable camTable =
          NetworkTableInstance.getDefault().getTable("photonvision/" + cameras[i].getName());
      NetworkTableEntry heartbeatEntry = camTable.getEntry("heartbeat");

      if (!heartbeatEntry.exists()) {
        results.put(i, null);
        bestTarget.put(i, new ArrayList<>());
        System.err.println(cameraNames[i] + "doesn't have a heartbeat entry");
        continue;
      }

      long currentHeartbeat = (long) heartbeatEntry.getDouble(-1);
      if (currentHeartbeat == lastHeartbeats[i]) {
        // Heartbeat hasn't changed → camera likely stalled or unplugged
        results.put(i, null);
        bestTarget.put(i, new ArrayList<>());
        System.err.println(cameraNames[i] + "heartbeat stayed the same");
        continue;
      }

      lastHeartbeats[i] = currentHeartbeat;

      List<PhotonPipelineResult> unreadResults = cameras[i].getAllUnreadResults();
      // System.out.println(cameraNames[i] + "pipeline updated");
      if (!unreadResults.isEmpty()) {
        results.put(i, unreadResults);
      } else {
        System.err.println(cameraNames[i] + "pipeline is empty");
        continue;
      }
      if (!results.isEmpty()) {
        bestTarget.put(i, results.get(i).get(0).getTargets());
        // System.out.println("Best Target IS GETTING UPDATED -------------- for " +
        // cameraNames[i]);
      }
    }
  }

  private boolean isCameraConnected(String cameraName) {
    NetworkTable table = NetworkTableInstance.getDefault().getTable("photonvision/" + cameraName);
    return table.getKeys().size() > 0;
  }

  // public static void updateCamerasPoseEstimation(Swerve s_Swerve, SwerveDrivePoseEstimator
  // poseEstimator, double
  //   camTrustValue)
  //   {
  //     for(int i = 0; i < cameras.length; i ++)
  //     {
  //       Optional<EstimatedRobotPose> estimatedRobotPose;
  //       if(!unreadResults.isEmpty())
  //       {
  //         estimatedRobotPose = photonPoseEstimators[i].update(unreadResults.get(0));
  //       }
  //       else
  //       {
  //         estimatedRobotPose = Optional.empty();
  //       }
  //       if(estimatedRobotPose.isPresent())
  //       {
  //             System.out.println("updating");
  //             EstimatedRobotPose temp = estimatedRobotPose.get();
  //               poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(camTrustValue,
  // camTrustValue, Double.MAX_VALUE));
  //               poseEstimator.addVisionMeasurement(temp.estimatedPose.toPose2d(),
  // temp.timestampSeconds);

  //               // Field2d localTempField2d = new Field2d();
  //               if(i == 0)
  //               {
  //                   theFieldCam0.setRobotPose(temp.estimatedPose.toPose2d());
  //               }
  //               else if (i == 1)
  //               {
  //                   theFieldCam1.setRobotPose(temp.estimatedPose.toPose2d());
  //               }
  //               // SmartDashboard.putString("Vision Estimated Pose for camera " +  i,
  // localTempField2d.toString());
  //           }
  //           else
  //           {
  //               if(i == 0)
  //               {
  //                   theFieldCam0.setRobotPose(new Pose2d());
  //               }
  //               else if (i == 1)
  //               {
  //                   theFieldCam1.setRobotPose(new Pose2d());
  //               }
  //           }
  //           // final int index = i; //for lambda.
  //           // estimatedRobotPose.ifPresentOrElse(
  //           //     estimate ->
  //           //     {
  //           //         poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(camTrustValue,
  // camTrustValue, Double.MAX_VALUE));
  //           //         poseEstimator.addVisionMeasurement(estimate.estimatedPose.toPose2d(),
  // estimate.timestampSeconds);
  //           //         Field2d localTempField2d = new Field2d();
  //           //         localTempField2d.setRobotPose(estimate.estimatedPose.toPose2d());
  //           //         // SmartDashboard.putData("Vision Estimated Pose for camera " +  index,
  // localTempField2d);
  //           //     }
  //           //     ,
  //           //     ()->
  //           //     {
  //           //         Field2d localTempField2d = new Field2d();
  //           //         // SmartDashboard.putData("Vision Estimated Pose for camera " +  index,
  // localTempField2d);
  //           //     }
  //           // );
  //         //   SmartDashboard.putNumber(tableKey + i + "id", bestTarget.get(i).fiducialId);
  //         // SmartDashboard.putNumber(tableKey + i + "pitch", bestTarget.get(i).pitch);
  //         // SmartDashboard.putNumber(tableKey + i + "yaw", bestTarget.get(i).yaw);
  //         // SmartDashboard.putNumber(tableKey + i + "ambiguity",
  // bestTarget.get(i).poseAmbiguity);
  //         // SmartDashboard.putNumber(tableKey + i + "skew", bestTarget.get(i).skew);
  //         // Transform3d camToTar = bestTarget.get(i).getBestCameraToTarget();
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose x", camToTar.getX());
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose y", camToTar.getY());
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose z", camToTar.getZ());
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose measure x",
  // camToTar.getMeasureX().abs(Meters));
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose measure y",
  // camToTar.getMeasureY().abs(Meters));
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose measure z",
  // camToTar.getMeasureZ().abs(Meters));
  //           SmartDashboard.putData("Cam0 Vision Feedback", theFieldCam0);
  //           SmartDashboard.putData("Cam1 Vision feedback", theFieldCam1);
  //       }
  //   }

  public PhotonCamera[] getCameras() {
    return cameras;
  }

  public HashMap<Integer, List<PhotonPipelineResult>> getResults() {
    return results;
  }

  public HashMap<Integer, List<PhotonTrackedTarget>> getBestTargets() {
    return bestTarget;
  }

  public static synchronized PhotonVisionSubsystem getInstance(String[] cameraNames) {
    if (INSTANCE == null) {
      INSTANCE = new PhotonVisionSubsystem(cameraNames);
    }
    return INSTANCE;
  }
}
