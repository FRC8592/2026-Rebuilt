package frc.robot.subsystems.swerve;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.path.GoalEndState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.List;

public class SwervePathPlanner {
    private final Swerve swerve;
    private final RobotConfig config;

    public SwervePathPlanner(Swerve swerve, RobotConfig config){
        this.swerve = swerve;
        this.config = config;

        // Optional: Configure AutoBuilder if you want centralized path management
        AutoBuilder.configure(
            swerve::getCurrentOdometryPosition, // Robot pose supplier
            swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
            //TODO: tune the pid constants
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
            ),

            config, // The robot configuration

            () -> {
            // Boolean supplier that controls when the path will be mirrored for the red alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent()) {
                    return alliance.get() == DriverStation.Alliance.Red;
                }

                return false;
            },
            
            swerve // Reference to this subsystem to set requirements
            
        );
    }

    /**
     * A method by PathPlanner that allows you to run a path outside of autos
     * 
     * @param pathName name of path to run
     * @param choreoPath whether the path is from Choreo or not
     * @return a command to run the swerve along the path
     */
    public Command followPathCommand(String pathName, boolean choreoPath) {
        try{
            PathPlannerPath path;
            if(!choreoPath)
                path = PathPlannerPath.fromPathFile(pathName);
            else 
                path = PathPlannerPath.fromChoreoTrajectory(pathName);

            return new FollowPathCommand(
                    path,
                    swerve::getCurrentOdometryPosition, // Robot pose supplier
                    swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                    swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds, AND feedforwards
                    //TODO: tune the pid constants
                    new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                            new PIDConstants(5.0, 0.0, 0.0), // Translation PID constants
                            new PIDConstants(5.0, 0.0, 0.0) // Rotation PID constants
                    ),

                    config, // The robot configuration

                    () -> {
                    // Boolean supplier that controls when the path will be mirrored for the red alliance
                    // This will flip the path being followed to the red side of the field.
                    // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                        var alliance = DriverStation.getAlliance();
                        if (alliance.isPresent()) {
                            return alliance.get() == DriverStation.Alliance.Red;
                        }

                        return false;
                    },
                    
                    swerve // Reference to this subsystem to set requirements
            );

        } catch (Exception e) {
            DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
            return Commands.none();
        }
    }

    //TODO: don't let the robot drive into obstacles...
    public Command generatePath(List<Pose2d> poses){
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poses);

        //TODO: figure out how to calculate these values correctly
        PathConstraints constaints = new PathConstraints(
            null, 
            null, 
            null,
            null
        );
        
        PathPlannerPath path = new PathPlannerPath(
            waypoints, 
            constaints, 
            null, //null in order to start from your location on the field
            new GoalEndState(0, poses.get(poses.size() - 1).getRotation())
        );

        return AutoBuilder.followPath(path);
    }

    //TODO: does avoiding obstacles also mean avoiding the bump...?
    //TODO: the robot can't go under the bar thing
    /**
     * A method that returns a Command with a path for the robot to drive to the target pose. 
     * This method will avoid known field obstacles, but in rare cases could have odd behavior because 
     * the algorithm pathfinds while the robot moves. This method won't end with the desired robot heading, 
     * so it's reccomended that you use this to drive to an initial point of a path,  
     * then use that path to get the final pose with the desired robot heading.
     * @param targetPose the pose for the robot to move to
     * @return a command to drive the robot to that pose 
     */
    public Command pathfindToTarget(Pose2d targetPose){
        //TODO: calculate the correct values for the constraint
        PathConstraints constaints = new PathConstraints(
            null, 
            null, 
            null,
            null
        );

        return AutoBuilder.pathfindToPose(targetPose, constaints, 0.0);
    }

    public Command pathfindToTargetWithAlignment(PathPlannerPath path){
        PathConstraints constaints = new PathConstraints(
            null, 
            null, 
            null,
            null
        );

        return AutoBuilder.pathfindThenFollowPath(path, constaints);

    }
}
