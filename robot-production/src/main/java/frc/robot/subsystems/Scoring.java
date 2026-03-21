package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.Constants.MEASUREMENTS;
import frc.robot.Constants.SCORING;
import frc.robot.Constants.TURRET;
import frc.robot.subsystems.swerve.Swerve;

public class Scoring extends SubsystemBase {
    // Subsystems
    Swerve swerve; // Passed into the constructor so that we can get the current robot pose for
                   // tracking
    public Turret turret;
    public Shooter shooter;
    public Indexer indexer;
    public Intake intake;
    public LEDs leds;
    // Make tracking subsystems toggle on and off
    private boolean trackingTarget = false;
    private boolean overrideTracking = false;
    private double kFactor = 2.8; //extra velocity needed for flywheel
    private double kAdjustment = 0.1;
    private boolean targetIsHub;
    private Alliance alliance;

    /**
     * Scoring subsystem. Controls collecting and shooting.
     * 
     * @param swerve Newton swerve drive object
     */
    public Scoring(Swerve swerve, LEDs leds) {
        this.swerve = swerve;
        this.leds = leds;

        //
        // Instantiate subsystems
        //
        turret = new Turret();
        shooter = new Shooter();
        intake = new Intake();
        indexer = new Indexer();

        SmartDashboard.putNumber("shooterV", 0.0);
    }

    /**
     * Sets the alliance from the DriverStation
     * 
     * @param alliance Alliance
     */
    public void setAlliance(Alliance alliance) {
        this.alliance = alliance;
    }

    /**
     * Sets the target based on the robot's position on the field
     * 
     * @param currentRobotPose the robot's current position
     * @return the current target's Pose2d
     */
    public Pose2d getTarget(Pose2d currentRobotPose) {
        Pose2d targetPose = new Pose2d(0, 0, new Rotation2d(0));
        if (alliance == Alliance.Blue) {
            // if we're in our alliance zone
            if (currentRobotPose.getX() < (MEASUREMENTS.FIELD_X_METERS / 4) + 0.5) {
                targetPose = SCORING.BLUE_HUB_POSE;
                targetIsHub = true;
            }
            // if we're in the bottom half of the field
            else if (currentRobotPose.getY() < MEASUREMENTS.FIELD_Y_METERS / 2) {
                targetPose = SCORING.BLUE_PASSING_LOW_POSE;
                targetIsHub = false;
            } else {
                targetPose = SCORING.BLUE_PASSING_HIGH_POSE;
                targetIsHub = false;
            }
        } else {
            // if we're in our alliance zone
            if (currentRobotPose.getX() > (MEASUREMENTS.FIELD_X_METERS * (3 / 4)) - 0.5) {
                targetPose = SCORING.RED_HUB_POSE;
                targetIsHub = true;
            }
            // if we're in the bottom half of the field
            else if (currentRobotPose.getY() < MEASUREMENTS.FIELD_Y_METERS / 2) {
                targetPose = SCORING.RED_PASSING_LOW_POSE;
                targetIsHub = false;
            } else {
                targetPose = SCORING.RED_PASSING_HIGH_POSE;
                targetIsHub = false;
            }
        }
        return targetPose;
    }

    /**
     * Toggle the Tracking system on and off. The tracking system includes the turret angle and the
     * shooter speed
     */
    private void toggleTracking() {
        trackingTarget = !trackingTarget;
        overrideTracking = false;
    }

    public void overrideTracking() {
        trackingTarget = false;
        overrideTracking = true;
        turret.holdPosition();
        shooter.runAtSpeed(5900);
    }

    public Command overrideTrackingCommand() {
        return this.runOnce(() -> overrideTracking());
    }

    /**
     * Turn the tracking system off.
     */
    public void disableTracking() {
        trackingTarget = false;
        overrideTracking = false;
        turret.stop();
        shooter.stop();
    }

    /**
     * Command to run the intake at a set speed Just pass the command from Intake up to the next
     * level
     */
    // public Command runAtSpeedIntakeCommand() {
    // return intake.runAtSpeedIntakeCommand();
    // }

    /**
     * Command to run the indexer at a set speed Just pass the command from Indexer up to the next
     * level
     */
    public Command runAtSpeedIndexerCommand() {
        // if (shooter.getVelocityFlywheel() > SCORING.SHOOTER_THRESHOLD) {
        // return indexer.runIndexerCommand();
        // }
        return indexer.runIndexerCommand();
    }

    /**
     * Command to toggle turret tracking and shooter wheel speed
     */
    public Command toggleTrackingCommand() {
        return this.runOnce(() -> toggleTracking());
    }

    /**
     * Command to turn off turret tracking and shooter wheel speed
     */
    public Command disableTrackingCommand() {
        return this.runOnce(() -> disableTracking());
    }

    /**
     * 
     * @return
     */
    public boolean canShoot() {
        // TODO: Change so it can use blue or red hub tracking
        return Math.abs(turret.getAngle() - turret.calcAngle(swerve.getCurrentOdometryPosition(),
                getTarget(swerve.getCurrentOdometryPosition()))) <= TURRET.TURRET_TOLERANCE
        // && Math.abs(shooter.getVelocityFlywheel() -
        // RangeTable.get(swerve.getCurrentOdometryPosition().getTranslation().getDistance(getTarget(swerve.getCurrentOdometryPosition()).getTranslation()),
        // targetIsHub)) <= SHOOTER.SHOOTER_TOLERANCE
        ;
    }

    // double initialAngle = 62; //degrees
    // double hubHeight = 6.5; //feet
    // double initialBallHeight = 2.18; //feet
    // double g = 32.174; //feet per s^2
    // double flywheelRadius = 2.0; //inches
    // double flywheelGearing = 1.0;
    // double feetPerMeter = 3.28084;
    // public double shooterSpeedHub(double targetDistance) {
    //     double kFactor = SmartDashboard.getNumber("kFactor", 2.8); //extra velocity needed for flywheel
    //     double kAdjustment = SmartDashboard.getNumber("kAdjustment",  0.1);
    //     double AdjustedK = kFactor + kAdjustment * targetDistance;
    //     double distanceFeet = targetDistance * feetPerMeter;
    //     double angleRadians = initialAngle * Math.PI/180.0;
    //     double denominator = initialBallHeight+Math.tan(angleRadians)*distanceFeet-hubHeight;
    //     if (denominator<=0) return 0;
    //     double initialBallVelocity = Math.sqrt(distanceFeet*distanceFeet*g/(2.0*(initialBallHeight+Math.tan(angleRadians)*distanceFeet-hubHeight))) / Math.cos(angleRadians);
    //     double flyRadiusFeet = flywheelRadius / 12.0;
    //     double outputRPM = AdjustedK*(initialBallVelocity/flyRadiusFeet)*(60.0/(2*Math.PI));
    //     return outputRPM*flywheelGearing;
    // }

    /**
     * If the tracking system is toggled on, update the required turret angle and shooter speed
     */
    @Override
    public void periodic() {
        double targetDistance;
        double shooterSpeed;

        // Current robot pose and target pose
        Pose2d currentRobotPose = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d currentTargetPose = SCORING.BLUE_HUB_POSE;
        Logger.recordOutput(SCORING.LOG_PATH + "kFactor", kFactor);
        Logger.recordOutput(SCORING.LOG_PATH + "kAdjustment", kAdjustment);
        Logger.recordOutput(SCORING.LOG_PATH + "Tracking", trackingTarget);

        // get the current robot position and select the target
        currentRobotPose = swerve.getCurrentOdometryPosition();
        currentTargetPose = getTarget(currentRobotPose);

        Logger.recordOutput(SCORING.LOG_PATH + "target", currentTargetPose);

        if (indexer.indexerRunning) {
            leds.displayindexerRunning();
        }

        if (trackingTarget) {
            if (indexer.indexerRunning) {
                leds.displayindexerRunning();
            } else if (canShoot()) {
                leds.setCanShoot();
            } else {
                leds.setCannotShoot();
            }

            // calculate the distance to the target position
            targetDistance = currentRobotPose.getTranslation()
                    .getDistance(currentTargetPose.getTranslation());

            // Lookup the required shooter speed in the range table
            shooterSpeed = RangeTable.get(targetDistance, targetIsHub);
           // shooterSpeed = shooterSpeedHub(targetDistance);
            shooterSpeed = SmartDashboard.getNumber("V Flywheel", 0.0);

            // Log the current distance-to-target and shooter speed for debugging
            Logger.recordOutput(SCORING.LOG_PATH + "Target Distance", targetDistance);
            Logger.recordOutput(SCORING.LOG_PATH + "Shooter Speed", shooterSpeed); // rotations per
                                                                                   // second

            // Update turret angle and shooter speed
            turret.TurrettoAngle(currentRobotPose, currentTargetPose);
            shooter.runAtSpeed(shooterSpeed);
        } else {
            // Shut down the shooter motors. The turret will hold the last position, so we
            // don't need to send any command to it.
            if (!overrideTracking && !DriverStation.isDisabled() && !indexer.indexerRunning) {
                leds.setOff();
                shooter.stop();
            }
        }
    }

}
