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
import frc.robot.Constants.SCORING;
import frc.robot.subsystems.Scoring;
import frc.robot.subsystems.swerve.Swerve;

public class GamePieceSim extends SubsystemBase {
    private static final String LOG_PATH = "Simulation/GamePieces/";
    private static final double COLLECTION_RADIUS_METERS = 0.55;
    private static final double INTAKE_FORWARD_OFFSET_METERS = 0.55;
    private static final double BALL_RADIUS_METERS = 0.12;
    private static final double HELD_BALL_HEIGHT_METERS = 0.35;
    private static final double SHOT_BALL_HEIGHT_METERS = 1.1;
    private static final double SHOT_TIME_SECONDS = 0.75;
    private static final int ROBOT_CAPACITY = 2;

    private final Swerve swerve;
    private final Scoring scoring;
    private final List<Pose2d> visibleBalls = new ArrayList<>();

    private int heldBalls = 1;
    private boolean wasIndexerRunning = false;
    private boolean shotActive = false;
    private double shotStartTime = 0.0;
    private Pose2d shotStartPose = new Pose2d();
    private Pose2d shotEndPose = SCORING.BLUE_HUB_POSE;

    public GamePieceSim(Swerve swerve, Scoring scoring) {
        this.swerve = swerve;
        this.scoring = scoring;
        resetBalls();
    }

    public void resetBalls() {
        visibleBalls.clear();
        visibleBalls.add(new Pose2d(2.3, 5.5, new Rotation2d()));
        visibleBalls.add(new Pose2d(2.6, 2.6, new Rotation2d()));
        visibleBalls.add(new Pose2d(5.9, 5.4, new Rotation2d()));
        visibleBalls.add(new Pose2d(5.9, 2.7, new Rotation2d()));
        visibleBalls.add(new Pose2d(8.2, 4.0, new Rotation2d()));
        visibleBalls.add(new Pose2d(10.7, 5.4, new Rotation2d()));
        visibleBalls.add(new Pose2d(10.7, 2.7, new Rotation2d()));
        heldBalls = 1;
        wasIndexerRunning = false;
        shotActive = false;
    }

    @Override
    public void periodic() {
        if (!RobotBase.isSimulation()) {
            return;
        }

        Pose2d robotPose = swerve.getCurrentOdometryPosition();
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

        for (int i = 0; i < visibleBalls.size(); i++) {
            if (intakePose.getDistance(visibleBalls.get(i).getTranslation())
                    <= COLLECTION_RADIUS_METERS) {
                visibleBalls.remove(i);
                heldBalls++;
                return;
            }
        }
    }

    private void updateShot(Pose2d robotPose) {
        boolean indexerRunning = scoring.indexer.isRunning();

        if (indexerRunning && !wasIndexerRunning && heldBalls > 0) {
            heldBalls--;
            shotActive = true;
            shotStartTime = Timer.getFPGATimestamp();
            shotStartPose = robotPose;
            shotEndPose = getNearestHub(robotPose);
        }

        if (shotActive && Timer.getFPGATimestamp() - shotStartTime > SHOT_TIME_SECONDS) {
            shotActive = false;
        }

        wasIndexerRunning = indexerRunning;
    }

    private Pose2d getNearestHub(Pose2d robotPose) {
        double blueDistance =
                robotPose.getTranslation().getDistance(SCORING.BLUE_HUB_POSE.getTranslation());
        double redDistance =
                robotPose.getTranslation().getDistance(SCORING.RED_HUB_POSE.getTranslation());
        return blueDistance <= redDistance ? SCORING.BLUE_HUB_POSE : SCORING.RED_HUB_POSE;
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
        Pose2d[] heldBallPoses = new Pose2d[heldBalls];

        for (int i = 0; i < heldBalls; i++) {
            Translation2d offset = new Translation2d(-0.2 - (0.15 * i), robotPose.getRotation());
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
}
