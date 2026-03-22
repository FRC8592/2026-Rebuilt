package frc.robot.subsystems;

import java.util.function.DoubleUnaryOperator;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
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
    private double kFactor = 1.9; //extra velocity needed for flywheel
    private double kAdjustment = 0.42;
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

        SmartDashboard.putNumber("kFactor", 1.9);
        SmartDashboard.putNumber("kAdjustment", 0.42);
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
        double initialAngle = 64; //degrees
    double hubHeight = 6; //feet
    double initialBallHeight = 2.18; //feet
    double g = 32.174; //feet per s^2
    double flywheelRadius = 2.0; //inches
    double flywheelGearing = 1.0;
    double feetPerMeter = 3.28084;
    public double shooterSpeedHub(double targetDistance) {
        double kFactor = SmartDashboard.getNumber("kFactor", 1.9); //extra velocity needed for flywheel
        double kAdjustment = SmartDashboard.getNumber("kAdjustment", 0.42);
        double adjustedK = kFactor + kAdjustment * targetDistance;
        double distanceFeet = targetDistance * feetPerMeter;
        double angleRadians = initialAngle * Math.PI/180.0;
        double denominator = initialBallHeight+Math.tan(angleRadians)*distanceFeet-hubHeight;
        if (denominator<=0) return 0.0;
        double initialBallVelocity = Math.sqrt(distanceFeet*distanceFeet*g/(2.0*denominator)) / Math.cos(angleRadians);
        double flyRadiusFeet = flywheelRadius / 12.0;
        double outputRPM = adjustedK*(initialBallVelocity/flyRadiusFeet)*(60.0/(2*Math.PI));
        return (outputRPM*flywheelGearing);
    }

    public void increaseK() {
        kFactor += 0.1;
    }

    public void decreaseK() {
        kFactor -= 0.1;
    }

    public static double solveV0y(
            double thetaRad,
            double x,      // radial distance to target
            double vRx,    // robot tangential velocity
            double vRy,    // robot radial velocity toward target
            double hI,
            double hF,
            double g
    ) {
        double dh = hF - hI;
        double tanTheta = Math.tan(thetaRad);
    
        DoubleUnaryOperator f = (v0y) -> {
            double denom = v0y + vRy;
            if (denom <= 0.0) {
                return Double.NaN; // invalid physically
            }
    
            double horizSpeedRelativeRobot = Math.sqrt(v0y * v0y + vRx * vRx);
    
            return tanTheta * horizSpeedRelativeRobot * x / denom
                    - 0.5 * g * x * x / (denom * denom)
                    - dh;
        };
    
        // Start just above the singularity v0y = -vRy
        double lower = -vRy + 1e-9;
        if (lower <= 0.0) {
            lower = 1e-9;
        }
    
        // Search for a sign change by scanning upward.
        double upper = lower + Math.max(10.0, Math.abs(x) * 10.0 + 10.0);
        double left = lower;
        double fLeft = f.applyAsDouble(left);
    
        for (int expand = 0; expand < 20; expand++) {
            double prevU = left;
            double prevF = fLeft;
    
            int samples = 2000;
            for (int i = 1; i <= samples; i++) {
                double u = lower + (upper - lower) * i / samples;
                double fu = f.applyAsDouble(u);
    
                if (Double.isFinite(prevF) && Double.isFinite(fu)) {
                    if (prevF == 0.0) {
                        return prevU;
                    }
                    if (prevF * fu <= 0.0) {
                        return bisect(f, prevU, u, 1e-5, 100);
                    }
                }
    
                prevU = u;
                prevF = fu;
            }
    
            // No root in this interval; expand and try again.
            upper *= 2.0;
        }
    
        throw new IllegalArgumentException("No physical root found for v0y.");
    }
    
    private static double bisect(DoubleUnaryOperator f, double a, double b, double tol, int maxIter) {
        double fa = f.applyAsDouble(a);
        double fb = f.applyAsDouble(b);

        if (!Double.isFinite(fa) || !Double.isFinite(fb)) {
            throw new IllegalArgumentException("Invalid bracket endpoints.");
        }
        if (fa * fb > 0.0) {
            throw new IllegalArgumentException("Bisection requires a sign change.");
        }

        for (int i = 0; i < maxIter; i++) {
            double m = 0.5 * (a + b);
            double fm = f.applyAsDouble(m);

            if (!Double.isFinite(fm)) {
                // Nudge away from singularities if needed.
                m = Math.nextUp(m);
                fm = f.applyAsDouble(m);
            }

            if (Math.abs(fm) < tol || 0.5 * (b - a) < tol) {
                return m;
            }

            if (fa * fm <= 0.0) {
                b = m;
                fb = fm;
            } else {
                a = m;
                fa = fm;
            }
        }

        return 0.5 * (a + b);
    }

    /**
     * Shoot on the Move
     * Returns a pair of flywheel motor RPM and turret angle
     * Turret angle (degrees) is relative to the field, counterclockwise from x-axis
     * If no solutions exist, outputs (0, 0)
     */

    public Pair<Double, Double> SOTM(double targetX, double targetY, double robotVelX, double robotVelY) {
        double thetaDeg = initialAngle;
        double thetaRad = Math.toRadians(thetaDeg);
        double targetXFeet = targetX * feetPerMeter;
        double targetYFeet = targetY * feetPerMeter;

        double x = Math.sqrt(targetXFeet*targetXFeet + targetYFeet*targetYFeet);

        double angleToHub = Math.atan2(targetYFeet, targetXFeet);
        double vRx = robotVelX * Math.sin(angleToHub) + robotVelY * Math.cos(angleToHub);
        double vRy = robotVelX * Math.cos(angleToHub) + robotVelY * Math.sin(angleToHub);
        double hI = initialBallHeight;
        double hF = hubHeight;
        double g = 32.174;
        try {
            double v0y = solveV0y(thetaRad, x, vRx, vRy, hI, hF, g);
            double v0x = -vRx; // if tangential motion is being canceled
            double totalSpeedRelativeRobot = Math.sqrt(v0x * v0x + v0y * v0y)*Math.sqrt(1 + Math.tan(thetaRad)*Math.tan(thetaRad));
            double flyRadiusFeet = flywheelRadius / 12.0;
            double adjustedK = kFactor + kAdjustment*x/feetPerMeter;
            double outputRPM = adjustedK*(totalSpeedRelativeRobot/flyRadiusFeet)*(60.0/(2*Math.PI));

            double turretAngleToHub = Math.atan2(v0y, v0x);

            double turretFieldAngle = (((Math.toDegrees(turretAngleToHub) - (90.0 - Math.toDegrees(angleToHub))))%360+360)%360;

            return new Pair<>(outputRPM*flywheelGearing, turretFieldAngle);
        } catch (IllegalArgumentException e) {
            return new Pair<>(0.0, 0.0);
        }
    }

    /**
     * If the tracking system is toggled on, update the required turret angle and shooter speed
     * Shoot On The Move
     **/
    @Override
    public void periodic() {
        double targetDistance;
        double shooterSpeed;
        double targetX;
        double targetY;
        double turretAngle;

        // Current robot pose and target pose
        Pose2d currentRobotPose = new Pose2d(0, 0, new Rotation2d(0));
        Pose2d currentTargetPose = SCORING.BLUE_HUB_POSE;

        Logger.recordOutput(SCORING.LOG_PATH + "Tracking", trackingTarget);

        Logger.recordOutput(SCORING.LOG_PATH + "kFactor", kFactor);
        Logger.recordOutput(SCORING.LOG_PATH + "kAdjustment", kAdjustment);

        // get the current robot position and select the target
        currentRobotPose = swerve.getCurrentOdometryPosition();
        currentTargetPose = getTarget(currentRobotPose);

        Logger.recordOutput(SCORING.LOG_PATH + "target", currentTargetPose);

        if (canShoot()) {
            LEDs.setCanShoot(true);
        } else {
            LEDs.setCanShoot(false);
        }

        if (trackingTarget) {
            // calculate the distance to the target position
            targetDistance = currentRobotPose.getTranslation().getDistance(currentTargetPose.getTranslation());
            targetX = currentTargetPose.getX() - currentRobotPose.getX();
            targetY = currentTargetPose.getY() - currentRobotPose.getY();

            ChassisSpeeds velocityVector = swerve.getRobotRelativeSpeeds();
            ChassisSpeeds fieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(velocityVector, swerve.getYaw());

            Pair<Double, Double> SOTMResults = SOTM(targetX, targetY, fieldRelative.vxMetersPerSecond, fieldRelative.vyMetersPerSecond);
            shooterSpeed = SOTMResults.getFirst();
            turretAngle = SOTMResults.getSecond();
//            shooterSpeed = shooterSpeedHub(targetDistance);
            // shooterSpeed = SmartDashboard.getNumber("V Flywheel", 0.0);

            // Log the current distance-to-target and shooter speed for debugging
            Logger.recordOutput(SCORING.LOG_PATH + "Target Distance", targetDistance);
            Logger.recordOutput(SCORING.LOG_PATH + "Shooter Speed", shooterSpeed); //rotations per second

            // Update turret angle and shooter speed
            turret.TurrettoAngle(currentRobotPose, turretAngle);
            shooter.runAtSpeed(shooterSpeed);
        } else {
            // Shut down the shooter motors.  The turret will hold the last position, so we don't need to send any command to it.
            if (!overrideTracking) {
                shooter.stop();
            }
        }
    }


//     /**
//      * If the tracking system is toggled on, update the required turret angle and shooter speed
//      */
//     @Override
//     public void periodic() {
//         double targetDistance;
//         double shooterSpeed;

//         // Current robot pose and target pose
//         Pose2d currentRobotPose = new Pose2d(0, 0, new Rotation2d(0));
//         Pose2d currentTargetPose = SCORING.BLUE_HUB_POSE;
//         Logger.recordOutput(SCORING.LOG_PATH + "kFactor", kFactor);
//         Logger.recordOutput(SCORING.LOG_PATH + "kAdjustment", kAdjustment);
//         Logger.recordOutput(SCORING.LOG_PATH + "Tracking", trackingTarget);

//         // get the current robot position and select the target
//         currentRobotPose = swerve.getCurrentOdometryPosition();
//         currentTargetPose = getTarget(currentRobotPose);

//         Logger.recordOutput(SCORING.LOG_PATH + "target", currentTargetPose);

//         if (indexer.indexerRunning) {
//             leds.displayindexerRunning();
//         }

//         if (trackingTarget) {
//             if (indexer.indexerRunning) {
//                 leds.displayindexerRunning();
//             } else if (canShoot()) {
//                 leds.setCanShoot();
//             } else {
//                 leds.setCannotShoot();
//             }

//             // calculate the distance to the target position
//             targetDistance = currentRobotPose.getTranslation()
//                     .getDistance(currentTargetPose.getTranslation());

//             // Lookup the required shooter speed in the range table
//             shooterSpeed = RangeTable.get(targetDistance, targetIsHub);
//            //shooterSpeed = shooterSpeedHub(targetDistance);
//             //shooterSpeed = SmartDashboard.getNumber("V Flywheel", 0.0);

//             // Log the current distance-to-target and shooter speed for debugging
//             Logger.recordOutput(SCORING.LOG_PATH + "Target Distance", targetDistance);
//             Logger.recordOutput(SCORING.LOG_PATH + "Shooter Speed", shooterSpeed); // rotations per
//                                                                                    // second

//             // Update turret angle and shooter speed
//             turret.TurrettoAngle(currentRobotPose, currentTargetPose);
//             shooter.runAtSpeed(shooterSpeed);
//         } else {
//             // Shut down the shooter motors. The turret will hold the last position, so we
//             // don't need to send any command to it.
//             if (!overrideTracking && !DriverStation.isDisabled() && !indexer.indexerRunning) {
//                 leds.setOff();
//                 shooter.stop();
//             }
//         }
//     }

}
