package frc.robot.commands.largecommands;
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
import frc.robot.Robot;
import frc.robot.Constants.*;
import frc.robot.subsystems.swerve.Swerve;

public class FollowPathCommand extends LargeCommand {
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

    //check if red alliance when calling FollowPathCommand
    //true when red, false when blue
    private boolean isRedAlliance = DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get() == Alliance.Red;

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

    public FollowPathCommand(Optional<Trajectory> trajectory, double secondsPastPathEndTolerated, boolean rollAtPathEnd){
        this(trajectory);
        this.secondsPastPathEndTolerated = secondsPastPathEndTolerated;
        this.rollAtPathEnd = rollAtPathEnd;
    }

    public void initialize(){
        Logger.recordOutput("CustomLogs/CurrentPathCommand/Name", this.getName());

        if(this.trajectory.isPresent()) {
            Logger.recordOutput("CustomLogs/CurrentPathCommand/Trajectory", this.trajectory.get());
        } else {
            throw new Error("Could not find the trajectory " + trajectory + " provided in the constructor");
        }
        
        timer.reset();
        timer.start();

        if(isRedAlliance) { //if on red alliance, flip the intial position
            swerve.setKnownOdometryPose(trajectory.get().sample(0).poseMeters);  
        } else { //if on blue alliance, use the given pose from the trajectory without flipping
            swerve.setKnownOdometryPose(trajectory.get().sample(0).poseMeters);
        }
        
    }

    public void execute(){
        //if the trajectory has no paths
        if(trajectory.isEmpty()) {
            return;
        }

        // Instances of State contain information about pose, velocity, accelleration, curvature, etc.
        State desiredState = trajectory.get().sample(timer.get());

        if(isRedAlliance){
            desiredState = flip(desiredState);
            swerve.setKnownOdometryPose(desiredState.poseMeters);
        }

        Logger.recordOutput(SWERVE.LOG_PATH+"TargetPoseX", desiredState.poseMeters.getX());
        Logger.recordOutput(SWERVE.LOG_PATH + "TargetPoseY", desiredState.poseMeters.getY());
        Logger.recordOutput(SWERVE.LOG_PATH + "TargetPoseRot", desiredState.poseMeters.getRotation().getDegrees());
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
                MEASUREMENTS.FIELD_X_METERS + state.poseMeters.getX(),
                state.poseMeters.getY(),
                Rotation2d.fromRadians(Math.PI).minus(state.poseMeters.getRotation())
            ),
            -state.curvatureRadPerMeter
        );
    }

}