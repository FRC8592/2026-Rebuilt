package frc.robot.commands.largecommands;
import frc.robot.Robot;
import frc.robot.Constants.*;

import java.util.Optional;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;

public class FollowPathCommand extends LargeCommand {
    private static String LOG_PATH = "CustomLogs/CurrentPathCommand/";

    // Pathing variables
    private Optional<Trajectory> trajectory;
    private Timer timer = new Timer();

    // PID controllers
    private ProfiledPIDController turnController;
    private HolonomicDriveController drivePID;
    private PIDController xController;
    private PIDController yController;

    //check if red alliance when calling FollowPathCommand: true when red, false when blue
    private boolean isRedAlliance = false; 

    /**
     * Command to follow a trajectory
     *
     * @param trajectory the trajectory to follow on the red side of the field.
     */
    public FollowPathCommand(Optional<Trajectory> trajectory){
        super(swerve);

        this.trajectory = trajectory;

        this.xController = new PIDController(
            SWERVE.PATH_FOLLOW_TRANSLATE_kP,
            SWERVE.PATH_FOLLOW_TRANSLATE_kI,
            SWERVE.PATH_FOLLOW_TRANSLATE_kD
        );

        this.xController.setTolerance(
            SWERVE.PATH_FOLLOW_TRANSLATE_POSITION_TOLERANCE,
            SWERVE.PATH_FOLLOW_TRANSLATE_VELOCITY_TOLERANCE
        );

        this.yController = new PIDController(
            SWERVE.PATH_FOLLOW_TRANSLATE_kP,
            SWERVE.PATH_FOLLOW_TRANSLATE_kI,
            SWERVE.PATH_FOLLOW_TRANSLATE_kD
        );
        
        this.yController.setTolerance(
            SWERVE.PATH_FOLLOW_TRANSLATE_POSITION_TOLERANCE,
            SWERVE.PATH_FOLLOW_TRANSLATE_VELOCITY_TOLERANCE
        );

        this.turnController = new ProfiledPIDController(
            SWERVE.PATH_FOLLOW_ROTATE_kP,
            SWERVE.PATH_FOLLOW_ROTATE_kI,
            SWERVE.PATH_FOLLOW_ROTATE_kD,
            new Constraints(
                SWERVE.PATH_FOLLOW_ROTATE_MAX_VELOCITY,
                SWERVE.PATH_FOLLOW_ROTATE_MAX_ACCELLERATION
            )
        );

        this.turnController.setTolerance(
            SWERVE.PATH_FOLLOW_ROTATE_POSITION_TOLERANCE,
            SWERVE.PATH_FOLLOW_ROTATE_VELOCITY_TOLERANCE
        );

        this.turnController.enableContinuousInput(-Math.PI, Math.PI);

        this.drivePID = new HolonomicDriveController(xController, yController, turnController);
        this.drivePID.setTolerance(
            new Pose2d(
                new Translation2d(xController.getErrorTolerance(), yController.getErrorTolerance()),
                new Rotation2d(turnController.getPositionTolerance())
            )
        );
    }

    /**
     * Intiialize the FollowPathCommand with the provided trajectory in th constructor
     */
    public void initialize(){
        isRedAlliance = (DriverStation.getAlliance().isPresent() 
                    && DriverStation.getAlliance().get() == Alliance.Red);

        //TODO: make this show the name of the choreo trajectory that its running
        Logger.recordOutput(LOG_PATH + "PathName", this.getName());

        if (trajectory.isEmpty()) {
            throw new Error("FollowPathCommand initialized with empty trajectory");
        }

        timer.reset();
        timer.start();

        State initial = trajectory.get().sample(0.0);
        State startPose = isRedAlliance ? flip(initial) : initial;

        swerve.resetPose(startPose.poseMeters);
        Logger.recordOutput(LOG_PATH + "intialX", startPose.poseMeters.getX());
        Logger.recordOutput(LOG_PATH + "initialY", startPose.poseMeters.getY());
        Logger.recordOutput(LOG_PATH + "intialRot", startPose.poseMeters.getRotation().getRadians());

        //reset so that values aren't reused
        xController.reset();
        yController.reset();
        turnController.reset(swerve.getPose().getRotation().getRadians());

        Logger.recordOutput(LOG_PATH + "flipForRed", isRedAlliance);
        
    }

    /**
     * Run periodically until the trajectory is done
     */
    public void execute(){
        //if the trajectory has no paths
        if(trajectory.isEmpty())
            return;

        //Instances of State contain information about pose, velocity, accelleration, curvature, etc.
        State desiredState = trajectory.get().sample(timer.get());

        if(isRedAlliance)
            desiredState = flip(desiredState);

        Logger.recordOutput(LOG_PATH + "TargetPoseX", desiredState.poseMeters.getX());
        Logger.recordOutput(LOG_PATH + "TargetPoseY", desiredState.poseMeters.getY());
        Logger.recordOutput(LOG_PATH + "TargetPoseRot", desiredState.poseMeters.getRotation().getRadians());

        //robot-relative ChassisSpeeds object 
        ChassisSpeeds driveSpeeds = drivePID.calculate(
            swerve.getPose(),
            desiredState,
            desiredState.poseMeters.getRotation()
        );

        //negate y to ensure correct path following
        driveSpeeds = new ChassisSpeeds(
            driveSpeeds.vxMetersPerSecond,
            -driveSpeeds.vyMetersPerSecond,
            driveSpeeds.omegaRadiansPerSecond
        );

        swerve.drive(driveSpeeds);
    }

    public void end(boolean interrupted){
        Logger.recordOutput("CustomLogs/CurrentPathCommand/Name", "None");
    }

    /**
     * Returns whether the FollowPathCommand is finished or not
     * 
     * @return if finished
     */
    public boolean isFinished(){
        return ( // Only return true if enough time has elapsed, we're at the target location, and we're not using alternate movement.
            (
                trajectory.isEmpty() || (
                timer.hasElapsed(trajectory.get().getTotalTimeSeconds())
                && (drivePID.atReference() || !Robot.isReal())
            ))
        );
    }

    /**
     * Mirror a State object to the other side of the field.
     *
     * @param state {@code State}: the state to mirror
     * @return the mirrored state
     */
    private State flip(State state){
        return new State(
            state.timeSeconds,
            state.velocityMetersPerSecond,
            state.accelerationMetersPerSecondSq,
            new Pose2d(
                MEASUREMENTS.FIELD_X_METERS - state.poseMeters.getX(),
                state.poseMeters.getY(),
                Rotation2d.fromRadians(Math.PI - state.poseMeters.getRotation().getRadians())
            ),
            -state.curvatureRadPerMeter
        );
    }
}