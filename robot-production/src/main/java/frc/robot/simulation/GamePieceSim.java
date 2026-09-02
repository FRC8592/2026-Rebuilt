package frc.robot.simulation;

import java.util.ArrayList;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MEASUREMENTS;
import frc.robot.Constants.SCORING;
import frc.robot.subsystems.Scoring;
import frc.robot.subsystems.swerve.Swerve;

public class GamePieceSim extends SubsystemBase {
    private static final String LOG_PATH = "Simulation/GamePieces/";
    private static final double COLLECTION_RADIUS_METERS = 0.75;
    private static final double INTAKE_FORWARD_OFFSET_METERS = 0.55;
    private static final double BALL_RADIUS_METERS = 0.12;
    private static final double BALL_DIAMETER_METERS = BALL_RADIUS_METERS * 2.0;
    private static final double HELD_BALL_HEIGHT_METERS = 0.35;
    private static final double SHOT_BALL_HEIGHT_METERS = 1.1;
    private static final double SHOT_TIME_SECONDS = 0.75;
    private static final double SECONDS_PER_SHOT = 0.45;
    private static final double ROBOT_PUSH_RADIUS_METERS = 0.9;
    private static final double INTAKE_SCATTER_RADIUS_METERS = 1.0;
    private static final double ROBOT_PUSH_SPEED_METERS_PER_SECOND = 1.8;
    private static final double INTAKE_PUSH_SPEED_METERS_PER_SECOND = 2.4;
    private static final double BALL_FRICTION_PER_SECOND = 2.2;
    private static final double BALL_RESTITUTION = 0.35;
    private static final double MAX_BALL_SPEED_METERS_PER_SECOND = 3.0;
    private static final int ROBOT_CAPACITY = 99;
    private static final int MAX_DISPLAYED_HELD_BALLS = 2;

    private final Swerve swerve;
    private final Scoring scoring;
    private final List<Pose2d> visibleBalls = new ArrayList<>();
    private final List<Translation2d> ballVelocities = new ArrayList<>();

    private int heldBalls = 1;
    private boolean wasIndexerRunning = false;
    private boolean shotActive = false;
    private double lastUpdateTime = 0.0;
    private double shotStartTime = 0.0;
    private double lastShotTime = -SECONDS_PER_SHOT;
    private Pose2d shotStartPose = new Pose2d();
    private Pose2d shotEndPose = SCORING.BLUE_HUB_POSE;

    public GamePieceSim(Swerve swerve, Scoring scoring) {
        this.swerve = swerve;
        this.scoring = scoring;
        resetBalls();
    }

    public void resetBalls() {
        visibleBalls.clear();
        ballVelocities.clear();
        addNeutralZoneFuel();
        addDepotFuel();
        addOutpostChuteFuel();
        heldBalls = 1;
        wasIndexerRunning = false;
        shotActive = false;
        lastUpdateTime = Timer.getFPGATimestamp();
        lastShotTime = -SECONDS_PER_SHOT;
    }

    @Override
    public void periodic() {
        if (!RobotBase.isSimulation()) {
            return;
        }

        Pose2d robotPose = swerve.getCurrentOdometryPosition();
        double now = Timer.getFPGATimestamp();
        double dtSeconds = Math.min(0.05, Math.max(0.0, now - lastUpdateTime));
        lastUpdateTime = now;

        updateBallPhysics(robotPose, dtSeconds);
        collectNearbyBalls(robotPose);
        updateShot(robotPose);
        logState(robotPose);
    }

    private void collectNearbyBalls(Pose2d robotPose) {
        if (!scoring.intake.isRollerRunningForward() || heldBalls >= ROBOT_CAPACITY) {
            return;
        }

        Translation2d intakePose = robotPose.getTranslation()
                .plus(new Translation2d(INTAKE_FORWARD_OFFSET_METERS, robotPose.getRotation()));

        for (int i = visibleBalls.size() - 1; i >= 0 && heldBalls < ROBOT_CAPACITY; i--) {
            if (intakePose.getDistance(visibleBalls.get(i).getTranslation())
                    <= COLLECTION_RADIUS_METERS) {
                visibleBalls.remove(i);
                ballVelocities.remove(i);
                heldBalls++;
            }
        }
    }

    private void updateBallPhysics(Pose2d robotPose, double dtSeconds) {
        if (dtSeconds <= 0.0) {
            return;
        }

        Translation2d robotTranslation = robotPose.getTranslation();
        Translation2d intakeTranslation = robotTranslation
                .plus(new Translation2d(INTAKE_FORWARD_OFFSET_METERS, robotPose.getRotation()));

        for (int i = 0; i < visibleBalls.size(); i++) {
            Translation2d ballTranslation = visibleBalls.get(i).getTranslation();
            Translation2d velocity = ballVelocities.get(i);

            velocity = velocity.plus(getPushVelocity(ballTranslation, robotTranslation,
                    ROBOT_PUSH_RADIUS_METERS, ROBOT_PUSH_SPEED_METERS_PER_SECOND));

            if (scoring.intake.isRollerRunningForward()) {
                velocity = velocity.plus(getPushVelocity(ballTranslation, intakeTranslation,
                        INTAKE_SCATTER_RADIUS_METERS, INTAKE_PUSH_SPEED_METERS_PER_SECOND));
            }

            velocity = limitVelocity(velocity);
            ballTranslation = ballTranslation.plus(velocity.times(dtSeconds));

            double dampening = Math.max(0.0, 1.0 - (BALL_FRICTION_PER_SECOND * dtSeconds));
            velocity = velocity.times(dampening);

            BoundaryResult boundaryResult = clampToField(ballTranslation, velocity);
            visibleBalls.set(i, new Pose2d(boundaryResult.position, new Rotation2d()));
            ballVelocities.set(i, boundaryResult.velocity);
        }

        separateOverlappingBalls();
    }

    private Translation2d getPushVelocity(Translation2d ballTranslation, Translation2d source,
            double radiusMeters, double maxSpeedMetersPerSecond) {
        Translation2d delta = ballTranslation.minus(source);
        double distance = delta.getNorm();

        if (distance <= 0.001 || distance > radiusMeters) {
            return new Translation2d();
        }

        double strength = 1.0 - (distance / radiusMeters);
        return delta.div(distance).times(maxSpeedMetersPerSecond * strength);
    }

    private Translation2d limitVelocity(Translation2d velocity) {
        double speed = velocity.getNorm();

        if (speed <= MAX_BALL_SPEED_METERS_PER_SECOND) {
            return velocity;
        }

        return velocity.div(speed).times(MAX_BALL_SPEED_METERS_PER_SECOND);
    }

    private BoundaryResult clampToField(Translation2d position, Translation2d velocity) {
        double x = position.getX();
        double y = position.getY();
        double vx = velocity.getX();
        double vy = velocity.getY();

        if (x < BALL_RADIUS_METERS) {
            x = BALL_RADIUS_METERS;
            vx = Math.abs(vx) * BALL_RESTITUTION;
        } else if (x > MEASUREMENTS.FIELD_X_METERS - BALL_RADIUS_METERS) {
            x = MEASUREMENTS.FIELD_X_METERS - BALL_RADIUS_METERS;
            vx = -Math.abs(vx) * BALL_RESTITUTION;
        }

        if (y < BALL_RADIUS_METERS) {
            y = BALL_RADIUS_METERS;
            vy = Math.abs(vy) * BALL_RESTITUTION;
        } else if (y > MEASUREMENTS.FIELD_Y_METERS - BALL_RADIUS_METERS) {
            y = MEASUREMENTS.FIELD_Y_METERS - BALL_RADIUS_METERS;
            vy = -Math.abs(vy) * BALL_RESTITUTION;
        }

        return new BoundaryResult(new Translation2d(x, y), new Translation2d(vx, vy));
    }

    private void separateOverlappingBalls() {
        for (int i = 0; i < visibleBalls.size(); i++) {
            for (int j = i + 1; j < visibleBalls.size(); j++) {
                Translation2d first = visibleBalls.get(i).getTranslation();
                Translation2d second = visibleBalls.get(j).getTranslation();
                Translation2d delta = second.minus(first);
                double distance = delta.getNorm();

                if (distance <= 0.001 || distance >= BALL_DIAMETER_METERS) {
                    continue;
                }

                Translation2d direction = delta.div(distance);
                Translation2d correction =
                        direction.times((BALL_DIAMETER_METERS - distance) / 2.0);

                visibleBalls.set(i, new Pose2d(first.minus(correction), new Rotation2d()));
                visibleBalls.set(j, new Pose2d(second.plus(correction), new Rotation2d()));
            }
        }
    }

    private void updateShot(Pose2d robotPose) {
        boolean indexerRunning = scoring.indexer.isRunning();

        double now = Timer.getFPGATimestamp();

        if (indexerRunning && heldBalls > 0
                && (!wasIndexerRunning || now - lastShotTime >= SECONDS_PER_SHOT)) {
            shootHeldBall(robotPose, now);
        }

        if (shotActive && now - shotStartTime > SHOT_TIME_SECONDS) {
            shotActive = false;
        }

        wasIndexerRunning = indexerRunning;
    }

    private void shootHeldBall(Pose2d robotPose, double now) {
        heldBalls--;
        shotActive = true;
        shotStartTime = now;
        lastShotTime = now;
        shotStartPose = robotPose;
        shotEndPose = getNearestHub(robotPose);
    }

    private Pose2d getNearestHub(Pose2d robotPose) {
        double blueDistance =
                robotPose.getTranslation().getDistance(SCORING.BLUE_HUB_POSE.getTranslation());
        double redDistance =
                robotPose.getTranslation().getDistance(SCORING.RED_HUB_POSE.getTranslation());
        return blueDistance <= redDistance ? SCORING.BLUE_HUB_POSE : SCORING.RED_HUB_POSE;
    }

    private void addNeutralZoneFuel() {
        double centerX = MEASUREMENTS.FIELD_X_METERS / 2.0;
        double centerY = MEASUREMENTS.FIELD_Y_METERS / 2.0;

        addFuelGrid(centerX, centerY, 12, 18, BALL_DIAMETER_METERS, BALL_DIAMETER_METERS);
    }

    private void addDepotFuel() {
        addFuelGrid(3.65, MEASUREMENTS.FIELD_Y_METERS - 1.25, 6, 4,
                BALL_DIAMETER_METERS, BALL_DIAMETER_METERS);
        addFuelGrid(MEASUREMENTS.FIELD_X_METERS - 3.65, 1.25, 6, 4,
                BALL_DIAMETER_METERS, BALL_DIAMETER_METERS);
    }

    private void addOutpostChuteFuel() {
        addFuelGrid(0.95, MEASUREMENTS.FIELD_Y_METERS - 0.85, 6, 4,
                BALL_DIAMETER_METERS, BALL_DIAMETER_METERS);
        addFuelGrid(MEASUREMENTS.FIELD_X_METERS - 0.55,
                MEASUREMENTS.FIELD_Y_METERS - 0.85, 6, 4,
                BALL_DIAMETER_METERS, BALL_DIAMETER_METERS);
        addFuelGrid(0.55, 0.85, 6, 4, BALL_DIAMETER_METERS, BALL_DIAMETER_METERS);
        addFuelGrid(MEASUREMENTS.FIELD_X_METERS - 0.95, 0.85, 6, 4,
                BALL_DIAMETER_METERS, BALL_DIAMETER_METERS);
    }

    private void addFuelGrid(double centerX, double centerY, int columns, int rows, double spacingX,
            double spacingY) {
        double startX = centerX - ((columns - 1) * spacingX / 2.0);
        double startY = centerY - ((rows - 1) * spacingY / 2.0);

        for (int row = 0; row < rows; row++) {
            for (int column = 0; column < columns; column++) {
                double x = startX + (column * spacingX);
                double y = startY + (row * spacingY);
                addFuel(x, y);
            }
        }
    }

    private void addFuel(double x, double y) {
        visibleBalls.add(new Pose2d(x, y, new Rotation2d()));
        ballVelocities.add(new Translation2d());
    }

    private void logState(Pose2d robotPose) {
        Logger.recordOutput(LOG_PATH + "Visible Balls", visibleBalls.toArray(Pose2d[]::new));
        Logger.recordOutput(LOG_PATH + "Held Balls", getHeldBallPoses(robotPose));
        Logger.recordOutput(LOG_PATH + "Shot Ball", getShotBallPose());
        Logger.recordOutput(LOG_PATH + "Visible Balls 3D",
                toPose3dArray(visibleBalls.toArray(Pose2d[]::new), BALL_RADIUS_METERS));
        Logger.recordOutput(LOG_PATH + "Held Balls 3D",
                toPose3dArray(getHeldBallPoses(robotPose), HELD_BALL_HEIGHT_METERS));
        Logger.recordOutput(LOG_PATH + "Shot Ball 3D",
                toPose3dArray(getShotBallPose(), SHOT_BALL_HEIGHT_METERS));
        Logger.recordOutput(LOG_PATH + "Held Count", heldBalls);
        Logger.recordOutput(LOG_PATH + "Shot Active", shotActive);
    }

    private Pose2d[] getHeldBallPoses(Pose2d robotPose) {
        int displayedHeldBalls = Math.min(heldBalls, MAX_DISPLAYED_HELD_BALLS);
        Pose2d[] heldBallPoses = new Pose2d[displayedHeldBalls];

        for (int i = 0; i < displayedHeldBalls; i++) {
            Translation2d offset = new Translation2d(-0.18, 0.14 - (0.28 * i))
                    .rotateBy(robotPose.getRotation());
            heldBallPoses[i] = new Pose2d(robotPose.getTranslation().plus(offset),
                    robotPose.getRotation());
        }

        return heldBallPoses;
    }

    private Pose2d[] getShotBallPose() {
        if (!shotActive) {
            return new Pose2d[0];
        }

        double progress = Math.min(1.0, (Timer.getFPGATimestamp() - shotStartTime)
                / SHOT_TIME_SECONDS);
        Translation2d start = shotStartPose.getTranslation();
        Translation2d end = shotEndPose.getTranslation();
        Translation2d current = start.interpolate(end, progress);
        return new Pose2d[] {new Pose2d(current, shotStartPose.getRotation())};
    }

    private Pose3d[] toPose3dArray(Pose2d[] poses, double zMeters) {
        Pose3d[] poses3d = new Pose3d[poses.length];

        for (int i = 0; i < poses.length; i++) {
            poses3d[i] = new Pose3d(poses[i].getX(), poses[i].getY(), zMeters,
                    new Rotation3d(0.0, 0.0, poses[i].getRotation().getRadians()));
        }

        return poses3d;
    }

    private static class BoundaryResult {
        private final Translation2d position;
        private final Translation2d velocity;

        BoundaryResult(Translation2d position, Translation2d velocity) {
            this.position = position;
            this.velocity = velocity;
        }
    }
}
