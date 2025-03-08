package frc.robot;

import frc.robot.subsystems.Swerve;
import frc.robot.Constants;

import java.util.HashMap;

import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Voltage;

public class Autos {
    //TODO: fix the below values
    private static final double maxLinearVelo = 0;
    private static final double maxLinearAccel = 0;
    private static final double maxAngularVelo = 0;
    private static final double maxAngularAccel = 0;
    private static final double nominalVoltage = 12;
    private static final boolean unlimited = false;

    private static final PathConstraints constraints = new PathConstraints(maxLinearVelo, maxLinearAccel, maxAngularVelo, maxAngularAccel, nominalVoltage, unlimited);

    public static HashMap<String, PathPlannerPath> pathHashMap = new HashMap<String, PathPlannerPath>();

    //paths.put("Template Path", 
    PathPlannerPath template = new PathPlannerPath(
        PathPlannerPath.waypointsFromPoses(new Pose2d(0, 0, new Rotation2d(0)), new Pose2d(0, 0, new Rotation2d(0))),
        constraints,
        new IdealStartingState(0, new Rotation2d(0)),
        new GoalEndState(0, new Rotation2d(0)));

        //hHashMap.put("Template Path", template);

}