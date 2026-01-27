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

    private double secondsPastPathEndTolerated = -1;
    private boolean rollAtPathEnd = false;

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
                new Translation2d(xController.getPositionTolerance(), yController.getPositionTolerance()),
                new Rotation2d(turnController.getPositionTolerance())
            )
        );
    }

    /**
     * Command to follow a trajectory
     * 
     * @param trajectory the trajectory to follow 
     * @param secondsPastPathEndTolerated number of seconds 
     * @param rollAtPathEnd (?)
     */
    // public FollowPathCommand(Optional<Trajectory> trajectory, double secondsPastPathEndTolerated, boolean rollAtPathEnd){
    //     this(trajectory);
    //     this.secondsPastPathEndTolerated = secondsPastPathEndTolerated;
    //     this.rollAtPathEnd = rollAtPathEnd;
    // }

    /**
     * Intiialized the FollowPathCommand with the provided trajectory
     */
    public void initialize(){
        isRedAlliance = (DriverStation.getAlliance().isPresent() 
                    && DriverStation.getAlliance().get() == Alliance.Red);

        Logger.recordOutput(LOG_PATH + "PathName", this.getName());

        if (trajectory.isEmpty()) {
            throw new Error("FollowPathCommand initialized with empty trajectory");
        }

        timer.reset();
        timer.start();

        State initial = trajectory.get().sample(0.0);
        Pose2d startPose = isRedAlliance ? flip(initial).poseMeters : initial.poseMeters;

        swerve.setKnownOdometryPose(startPose);
        Logger.recordOutput(LOG_PATH + "intialX", startPose.getX());
        Logger.recordOutput(LOG_PATH + "initialY", startPose.getY());
        Logger.recordOutput(LOG_PATH + "intialRot", startPose.getRotation().getRadians());

        //will retain values from previous FollowPathCommand if not reset here and cause weird curving loopy paths
        xController.reset();
        yController.reset();
        turnController.reset(swerve.getCurrentOdometryPosition().getRotation().getRadians());

        Logger.recordOutput(LOG_PATH + "flipForRed", isRedAlliance);
        
    }

    /**
     * Run periodically until the trajectory is done
     */
    public void execute(){
        //if the trajectory has no paths
        if(trajectory.isEmpty())
            return;

        // Instances of State contain information about pose, velocity, accelleration, curvature, etc.
        State desiredState = trajectory.get().sample(timer.get());

        if(isRedAlliance)
            desiredState = flip(desiredState);

        Logger.recordOutput(LOG_PATH + "TargetPoseX", desiredState.poseMeters.getX());
        Logger.recordOutput(LOG_PATH + "TargetPoseY", desiredState.poseMeters.getY());
        Logger.recordOutput(LOG_PATH + "TargetPoseRot", desiredState.poseMeters.getRotation().getDegrees());
        // Logger.recordOutput(SWERVE.LOG_PATH+"TargetActualDifferenceX", desiredState.poseMeters.getX() - swerve.getCurrentOdometryPosition().getX());
        // Logger.recordOutput(SWERVE.LOG_PATH+"TargetActualDifferenceY", desiredState.poseMeters.getY() - swerve.getCurrentOdometryPosition().getY());
        // Logger.recordOutput(SWERVE.LOG_PATH+"TargetActualDifferenceRot", desiredState.poseMeters.getRotation().minus(swerve.getCurrentOdometryPosition().getRotation()).getDegrees());

        //robot-relative ChassisSpeeds object 
        ChassisSpeeds driveSpeeds = drivePID.calculate(
            swerve.getCurrentOdometryPosition(),
            desiredState,
            desiredState.poseMeters.getRotation()
        );

        swerve.drive(driveSpeeds);
    }

    public void end(boolean interrupted){
        Logger.recordOutput("CustomLogs/CurrentPathCommand/Name", "None");

        if(!rollAtPathEnd){
            swerve.drive(new ChassisSpeeds());
        }
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
                && (
                    (drivePID.atReference() || !Robot.isReal())
                )
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
                Rotation2d.fromRadians(Math.PI).minus(state.poseMeters.getRotation())
            ),

            -state.curvatureRadPerMeter
        );
    }

}