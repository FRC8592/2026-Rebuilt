package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.units.AngularAccelerationUnit;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.swerve.TunerConstants;

/**
 * Constants that can't be expressed in the .properties files: WPILib types, unit types,
 * and values computed from other constants. Everything else is generated into
 * {@link Constants} from src/main/deploy/*.properties.
 */
public final class CodeConstants {
  public final class MEASUREMENTS {
    public static final double FIELD_Y_METERS = 26.475 * Constants.CONVERSIONS.FEET_TO_METERS;
    public static final double FIELD_X_METERS = 54.2666667 * Constants.CONVERSIONS.FEET_TO_METERS;
  }

  public final class VISION {
    public static final String LOG_PATH = Constants.SHARED.LOG_FOLDER + "/Vision/";

    // camera G
    public static final Transform3d CAMERA_OFFSETS_RIGHT =
        (new Transform3d(new Translation3d(-0.07478,-0.33655,0.56), //x: -0.020817 z: 0.5653 y: -0.29
            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(270)))); //-0.334588, -0.072009, 0.5212334

    // camera F
    public static final Transform3d CAMERA_OFFSETS_BACK =
        (new Transform3d(new Translation3d(-0.08267,-0.28369,0.56), //0.2797302, -0.0772414, -0.5218176
            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(180)))); //x: -0.020817 z: 0.5653 y: -0.26

    // camera H
    public static final Transform3d CAMERA_OFFSETS_LEFT =
        (new Transform3d(new Translation3d(-0.07478, 0.33655,0.56),  //x: -0.020817 z: 0.5653 y: 0.29
            new Rotation3d(0, Math.toRadians(-15), Math.toRadians(90)))); //-0.334588, -0.072009, 0.5212334
  }

  public final class SWERVE {
    public static final String LOG_PATH = Constants.SHARED.LOG_FOLDER + "/SWERVE/";

    public static final TrajectoryConfig PATH_FOLLOW_TRAJECTORY_CONFIG =
        new TrajectoryConfig(4.5, 3);
    public static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(720);

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  }

  public final class SHOOTER {
    public static final String LOG_PATH = Constants.SHARED.LOG_FOLDER + "/SHOOTER/";
  }

  public final class TURRET {
    public static final Current CURRENT_LIMIT = Amps.of(50);
    public static final Voltage TURRET_P = Volts.of(40d); // 12;
    public static final Voltage TURRET_I = Volts.of(0.01);
    public static final Voltage TURRET_D = Volts.of(0.8); // 0.4;
    public static final Voltage TURRET_S = Volts.of(0.6);
    public static final Voltage TURRET_V = Volts.of(0d);
    public static final Voltage TURRET_A = Volts.of(0d);
    public static final double TURRET_TOTAL = Constants.TURRET.TURRET_G1 * Constants.TURRET.TURRET_G2;
    public static final double DEGREES_TO_MOTOR_ROTATIONS =
        (Constants.TURRET.TURRET_GT / Constants.TURRET.TURRET_GM) / 360d;

    //TODO: Check if this works
    public static final Velocity<AngularAccelerationUnit> MAX_JERK = RotationsPerSecondPerSecond.per(Second).of(3000);
    public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(300);
    public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(50);

    //TODO: Update these constants
    public static final Angle E1_OFFSET = Degrees.of(286);
    public static final Angle E2_OFFSET = Degrees.of(323.4);

    public static final String LOG_PATH = Constants.SHARED.LOG_FOLDER + "/TURRET/";
  }

  public static final class SCORING {
    public static final String LOG_PATH = Constants.SHARED.LOG_FOLDER + "/SCORING/";
    // targets
    public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.66, 4.04, new Rotation2d(0));
    public static final Pose2d BLUE_PASSING_LOW_POSE = new Pose2d(3.5, 2.02, new Rotation2d(0));
    public static final Pose2d BLUE_PASSING_HIGH_POSE = new Pose2d(3.5, 6.05, new Rotation2d(0));

    public static final Pose2d RED_HUB_POSE = new Pose2d(11.94, 4.04, new Rotation2d(0));
    public static final Pose2d RED_PASSING_LOW_POSE = new Pose2d(12.5, 2.02, new Rotation2d(0));
    public static final Pose2d RED_PASSING_HIGH_POSE = new Pose2d(12.5, 6.05, new Rotation2d(0));
  }

  public static class INTAKE {
    public static final String LOG_PATH = Constants.SHARED.LOG_FOLDER + "/Intake/";
  }

  public final class INDEXER {
    public static final String LOG_PATH = Constants.SHARED.LOG_FOLDER + "/Indexer/";
  }

  public final class LEDS {
    public static final Color RAINBOW = new Color(255, 0, 0); // (N/A)
    public static final Color HOT_PINK = new Color (255,29 ,206); // (N/A)
    public static final Color PRISMARINE = new Color(126, 171, 172); // (N/A)
    public static final Color TEAL = new Color(0, 64, 192); //Can shoot
    public static final Color ORANGE = new Color(192, 64, 0); // Cannot shoot
    public static final Color WHITE = new Color(255, 255, 255); // Indexer running forward (attempt to change to rainbow)
    public static final Color GREEN = new Color(0, 255, 0); // 2 or more apriltags detected
    public static final Color RED = new Color(255, 0, 0); // 0 apriltags detected
    public static final Color OFF = new Color(0, 0, 0); // LEDs off
    public static final Color YELLOW = new Color(255, 255, 0); // 1 apriltag detected
    public static final Color PURPLE = new Color(255, 0, 255); //dyerotor reverse (operator pressed button to reverse dyerotor [priotized over shooting])
    // Total number of LEDs including the CANdle LEDs
    public static final int FULL_LED_COUNT = Constants.LEDS.LED_STRIP_LENGTH + Constants.LEDS.LED_CANDLE_COUNT;
    public static final String LOG_PATH = Constants.SHARED.LOG_FOLDER + "/LEDS/";
  }
}
