package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.*;
import frc.robot.subsystems.swerve.Swerve;

public class Scoring extends SubsystemBase{
    // Subsystems
    Swerve swerve;          // Passed into the constructor so that we can get the current robot pose for tracking
    public Turret turret;
    public Shooter shooter;
    public Indexer indexer;
    public Intake intake;
    // Make tracking subsystems toggle on and off
    private boolean trackingTarget = false;
    
    double shooterSpeed = 0;

    private Alliance alliance;

    /**
     * Scoring subsystem.  Controls collecting and shooting.
     * @param swerve Newton swerve drive object
     */
    public Scoring(Swerve swerve){
        this.swerve = swerve;

        //
        // Instantiate subsystems
        //
        turret = new Turret();
        shooter = new Shooter();
        intake = new Intake();
        indexer = new Indexer();

        SmartDashboard.putNumber("ShootVel", shooterSpeed);
    }

    /**
     * Sets the alliance from the DriverStation
     * @param alliance Alliance
     */
    public void setAlliance(Alliance alliance)
    {
        this.alliance = alliance;
    }

    /**
     * Sets the target based on the robot's position on the field
     * @param currentRobotPose the robot's current position
     * @return the current target's Pose2d
     */
    public Pose2d getTarget(Pose2d currentRobotPose){
        Pose2d targetPose = new Pose2d(0, 0, new Rotation2d(0));
        if (alliance == Alliance.Blue){
            // if we're in our alliance zone
            if (currentRobotPose.getX() < MEASUREMENTS.FIELD_X_METERS / 4){
                targetPose = SCORING.BLUE_HUB_POSE;
            }
            // if we're in the bottom half of the field
            else if(currentRobotPose.getY() < MEASUREMENTS.FIELD_Y_METERS / 2){
                targetPose = SCORING.BLUE_PASSING_LOW_POSE;
            }
            else{
                targetPose = SCORING.BLUE_PASSING_HIGH_POSE;
            }
        }
        else{
            // if we're in our alliance zone
            if (currentRobotPose.getX() > MEASUREMENTS.FIELD_X_METERS * (3 / 4)){
                targetPose = SCORING.RED_HUB_POSE;
            }
            // if we're in the bottom half of the field
            else if(currentRobotPose.getY() < MEASUREMENTS.FIELD_Y_METERS / 2){
                targetPose = SCORING.RED_PASSING_LOW_POSE;
            }
            else{
                targetPose = SCORING.RED_PASSING_HIGH_POSE;
            }
        }
        return targetPose;
    }

    /**
     * Toggle the Tracking system on and off.
     * The tracking system includes the turret angle and the shooter speed
     */
    private void toggleTracking() {
        trackingTarget = !trackingTarget;
    }


    /**
     * Command to run the intake at a set speed
     * Just pass the command from Intake up to the next level
     */
    public Command runAtSpeedIntakeCommand() {
        return intake.runAtSpeedIntakeCommand();
    }


    /**
     * Command to toggle on turret tracking and shooter wheel speed
     */
    public Command toggleTrackingCommand() {
        return this.runOnce(() -> toggleTracking());
    }


    /**
     * If the tracking system is toggled on, update the required turret angle and shooter speed
     */
    @Override
    public void periodic(){
        double targetDistance;
        double shooterSpeed;

        // Current robot pose and target pose
        Pose2d currentRobotPose = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d currentTargetPose = SCORING.BLUE_HUB_POSE;

        Logger.recordOutput(SCORING.LOG_PATH +"Tracking", trackingTarget);

        // get the current robot position and select the target
        currentRobotPose = swerve.getCurrentOdometryPosition();
        currentTargetPose = getTarget(currentRobotPose);

        Logger.recordOutput(SCORING.LOG_PATH+"target", currentTargetPose);

        if (trackingTarget) {
            // calculate the distance to the target position
            targetDistance = currentRobotPose.getTranslation().getDistance(currentTargetPose.getTranslation());

            // Lookup the required shooter speed in the range table
            shooterSpeed = RangeTable.get(targetDistance);
            shooterSpeed = SmartDashboard.getNumber("ShootVel", shooterSpeed);

            // Log the current distance-to-target and shooter speed for debugging
            Logger.recordOutput(SCORING.LOG_PATH +"Target Distance", targetDistance);
            Logger.recordOutput(SCORING.LOG_PATH + "Shooter Speed", shooterSpeed); //rotations per second

            // Update turret angle and shooter speed
            turret.TurrettoAngle(currentRobotPose, currentTargetPose);
            shooter.runAtSpeed(shooterSpeed);
        }
        else {
            // Shut down the shooter motors.  The turret will hold the last position, so we don't need to send any command to it.
            shooter.stop();
        }
    }

}

