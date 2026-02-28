package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.SCORING;
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
    // Current robot pose and target pose
    private Pose2d currentRobotPose = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d currentTargetPose = SCORING.BLUE_HUB_POSE;
    

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
    }

    /**
     * Sets the alliance from the DriverStation
     * @param alliance Alliance
     */
    public void setAlliance(Alliance alliance)
    {
        this.alliance = alliance;
    }

    public void selectTarget(Pose2d currentRobotPose){

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

        Logger.recordOutput(SCORING.LOG_PATH +"Tracking", trackingTarget);

        if (trackingTarget) {
            // Get the current robot position and calculate the distance to the target position
            currentRobotPose = swerve.getCurrentOdometryPosition();
            targetDistance = currentRobotPose.getTranslation().getDistance(currentTargetPose.getTranslation());

            // Lookup the required shooter speed in the range table
            shooterSpeed = RangeTable.get(targetDistance);

            // Log the current distance-to-target and shooter speed for debugging
            Logger.recordOutput(SCORING.LOG_PATH +"Target Distance", targetDistance);
            Logger.recordOutput(SCORING.LOG_PATH + "Shooter Speed", shooterSpeed); //rotations per second

            // Update turret angle and shooter speed
            turret.TurrettoAngle(currentRobotPose, currentTargetPose);
            // shooter.runAtSpeed(shooterSpeed);
        }
        else {
            // Shut down the shooter motors.  The turret will hold the last position, so we don't need to send any command to it.
            shooter.stop();
        }
    }

}

