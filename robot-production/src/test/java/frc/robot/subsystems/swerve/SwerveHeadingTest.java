package frc.robot.subsystems.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class SwerveHeadingTest {
    @Test
    void calculatesHeadingFromRobotToFieldPoint() {
        Pose2d robotPose = new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(120.0));
        Pose2d targetPose = new Pose2d(4.0, 4.0, Rotation2d.kZero);

        Rotation2d heading = Swerve.calculateHeadingToPoint(robotPose, targetPose);

        assertEquals(45.0, heading.getDegrees(), 1e-9);
    }

    @Test
    void calculatesHeadingInAllAtan2Quadrants() {
        Pose2d robotPose = Pose2d.kZero;
        Pose2d targetPose = new Pose2d(-1.0, -1.0, Rotation2d.kZero);

        Rotation2d heading = Swerve.calculateHeadingToPoint(robotPose, targetPose);

        assertEquals(-135.0, heading.getDegrees(), 1e-9);
    }
}
