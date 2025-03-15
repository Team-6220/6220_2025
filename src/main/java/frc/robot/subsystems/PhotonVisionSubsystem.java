package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Meters;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;

public class PhotonVisionSubsystem extends SubsystemBase {
  //"src\\main\\java\\frc\\lib\\vision\\2025-reefscape-andymark.json"
  //"src\main\deploy\vision\2025-reefscape-andymark.json"
  // public static Path path = Filesystem.getDeployDirectory().toPath().resolve("2025-reefscape-andymark.json");
  private static PhotonCamera[] cameras =
  {
    new PhotonCamera("Bottom_Right_Cam"), //Top Right USB
    new PhotonCamera("Right_Ardu_Cam") //Bottom Right USB
    // new PhotonCamera("Left_Ardu_Cam")
  };

  public static Field2d theFieldCam0 = new Field2d(), theFieldCam1 = new Field2d();

  private PhotonTrackedTarget noErrorHopefully;

  private static PhotonVisionSubsystem INSTANCE = null;
  
  private static String tableKey = "Vision_";

  private HashMap<Integer, List<PhotonPipelineResult>> results;
  private HashMap<Integer, PhotonTrackedTarget> bestTarget;

  /** Creates a new PhotonVisionSubsystem. */
  public PhotonVisionSubsystem() {
    for (int i = 0; i < cameras.length; i++) {
      cameras[i].setPipelineIndex(0);
    }
  
    VisionConstants.setTagXYHeightAngle();

    results = new HashMap<Integer, List<PhotonPipelineResult>>();
    for (int i = 0; i < cameras.length; i++) {
      if (!cameras[i].getAllUnreadResults().isEmpty()) {
        results.put(i, cameras[i].getAllUnreadResults());
      }
      else
      {
        results.put(i, null);
      }
    }
    
    bestTarget = new HashMap<Integer, PhotonTrackedTarget>();
    for(int i = 0; i < cameras.length; i ++)
    {
      bestTarget.put(i, null);
    }

  }
  
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    for (int i = 0; i < cameras.length; i++) {
      List<PhotonPipelineResult> unreadResults = cameras[i].getAllUnreadResults();
      if (!unreadResults.isEmpty()) {
        results.put(i, unreadResults);
      }
      if (!results.get(i).isEmpty()) {
        bestTarget.put(i, results.get(i).get(0).getBestTarget());
        // System.out.println("Best Target IS GETTING UPDATED --------------");
        
        if(bestTarget.get(i) != null)
        {
          Transform3d camToTar = bestTarget.get(i).getBestCameraToTarget();
          SmartDashboard.putNumber(tableKey + i + "camera to pose x", camToTar.getX());
          SmartDashboard.putNumber(tableKey + i + "camera to pose y", camToTar.getY());
          SmartDashboard.putNumber(tableKey + i + "camera to pose z", camToTar.getZ());
          SmartDashboard.putNumber(tableKey + i + "camera to pose measure x", camToTar.getMeasureX().abs(Meters));
          SmartDashboard.putNumber(tableKey + i + "camera to pose measure y", camToTar.getMeasureY().abs(Meters));
          SmartDashboard.putNumber(tableKey + i + "camera to pose measure z", camToTar.getMeasureZ().abs(Meters));

          SmartDashboard.putNumber(tableKey + i + "id", bestTarget.get(i).fiducialId);
          SmartDashboard.putNumber(tableKey + i + "pitch", bestTarget.get(i).pitch);
          SmartDashboard.putNumber(tableKey + i + "yaw", bestTarget.get(i).yaw);
          SmartDashboard.putNumber(tableKey + i + "ambiguity", bestTarget.get(i).poseAmbiguity);
          SmartDashboard.putNumber(tableKey + i + "skew", bestTarget.get(i).skew);
        }
      }
    }
  }
  

  // public static void updateCamerasPoseEstimation(Swerve s_Swerve, SwerveDrivePoseEstimator poseEstimator, double 
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
  //               poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(camTrustValue, camTrustValue, Double.MAX_VALUE));
  //               poseEstimator.addVisionMeasurement(temp.estimatedPose.toPose2d(), temp.timestampSeconds);

  //               // Field2d localTempField2d = new Field2d();
  //               if(i == 0)
  //               {
  //                   theFieldCam0.setRobotPose(temp.estimatedPose.toPose2d());
  //               }
  //               else if (i == 1)
  //               {
  //                   theFieldCam1.setRobotPose(temp.estimatedPose.toPose2d());
  //               }
  //               // SmartDashboard.putString("Vision Estimated Pose for camera " +  i, localTempField2d.toString());
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
  //           //         poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(camTrustValue, camTrustValue, Double.MAX_VALUE));
  //           //         poseEstimator.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
  //           //         Field2d localTempField2d = new Field2d();
  //           //         localTempField2d.setRobotPose(estimate.estimatedPose.toPose2d());
  //           //         // SmartDashboard.putData("Vision Estimated Pose for camera " +  index, localTempField2d);
  //           //     }
  //           //     ,
  //           //     ()->
  //           //     {
  //           //         Field2d localTempField2d = new Field2d();
  //           //         // SmartDashboard.putData("Vision Estimated Pose for camera " +  index, localTempField2d);
  //           //     }
  //           // );
  //         //   SmartDashboard.putNumber(tableKey + i + "id", bestTarget.get(i).fiducialId);
  //         // SmartDashboard.putNumber(tableKey + i + "pitch", bestTarget.get(i).pitch);
  //         // SmartDashboard.putNumber(tableKey + i + "yaw", bestTarget.get(i).yaw);
  //         // SmartDashboard.putNumber(tableKey + i + "ambiguity", bestTarget.get(i).poseAmbiguity);
  //         // SmartDashboard.putNumber(tableKey + i + "skew", bestTarget.get(i).skew);
  //         // Transform3d camToTar = bestTarget.get(i).getBestCameraToTarget();
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose x", camToTar.getX());
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose y", camToTar.getY());
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose z", camToTar.getZ());
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose measure x", camToTar.getMeasureX().abs(Meters));
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose measure y", camToTar.getMeasureY().abs(Meters));
  //         // SmartDashboard.putNumber(tableKey + i + "camera to pose measure z", camToTar.getMeasureZ().abs(Meters));
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

  public HashMap<Integer, PhotonTrackedTarget> getBestTargets() {
    return bestTarget;
  }

  public static synchronized PhotonVisionSubsystem getInstance() {
    if (INSTANCE == null) {
        INSTANCE = new PhotonVisionSubsystem();
    }
    return INSTANCE;
  }
}