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
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.units.measure.Voltage;
import static edu.wpi.first.units.Units.*;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.subsystems.swerve.TunerConstants;

public final class Constants {
  public final class SHARED {
    public static final String LOG_FOLDER = "CustomLogs";
  }

  public final class MEASUREMENTS {
    public static final double FIELD_Y_METERS = 26.475 * CONVERSIONS.FEET_TO_METERS;
    public static final double FIELD_X_METERS = 54.2666667 * CONVERSIONS.FEET_TO_METERS;

  }

  public final class CONVERSIONS {
    public static final double METERS_TO_FEET = 3.28084;
    public static final double FEET_TO_METERS = 0.3048;
    public static final double INCHES_TO_METERS = 0.0254;
  }

  public final class CONTROLLERS {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
  }

  public final class VISION {
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Vision/";

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

    public static final String CAMERA_NAME_RIGHT = ("Right_Arducam_OV9782_G");

    public static final String CAMERA_NAME_BACK = ("Back_Arducam_OV9782_F");

    public static final String CAMERA_NAME_LEFT = ("Left_Arducam_OV9782_H");
    public static final double MAX_ACCEPTABLE_AMBIGUITY = 0.1;
    public static final double REJECT_SINGLE_TAG_POSE_ESTIMATE_RANGE = 3.5;
  }

  public final class SWERVE {
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/SWERVE/";

    // TODO: Double check that these PID constants still work
    public static final double SNAP_TO_kP = 3.2;
    public static final double SNAP_TO_kI = 0.0;
    public static final double SNAP_TO_kD = 0.1;

    public static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = 4.73;
    public static final TrajectoryConfig PATH_FOLLOW_TRAJECTORY_CONFIG =
        new TrajectoryConfig(4.5, 3);
    public static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(720);

    public static final double SIMULATED_STEER_INERTIA = 0.00001;
    public static final double SIMULATED_DRIVE_INERTIA = 0.06;
    public static final double SIMULATION_LOOP_PERIOD = 0.005;
    public static final double STEER_FRICTION_VOLTAGE = 0.25;
    public static final double DRIVE_FRICTION_VOLTAGE = 0.25;

    public static final double TRANSLATE_POWER_FAST = 1.0;
    public static final double ROTATE_POWER_FAST = 0.5;
    public static final double TRANSLATE_POWER_SLOW = 0.3;
    public static final double ROTATE_POWER_SLOW = 0.3;

    public static final int TRANSLATION_SMOOTHING_AMOUNT = 3;
    public static final int ROTATION_SMOOTHING_AMOUNT = 1;

    public static final double JOYSTICK_EXPONENT = 1.75;

    // swerve pid constants below
    public static final double STEER_KP = 51.925;
    public static final double STEER_KI = 0.0;
    public static final double STEER_KD = 3.00786;
    public static final double STEER_KS = 0.21866;
    public static final double STEER_KV = 2.5526;
    public static final double STEER_KA = 0.059517;

    public static final double DRIVE_KP = 0.11652;
    public static final double DRIVE_KI = 0.0;
    public static final double DRIVE_KD = 0.0;
    public static final double DRIVE_KS = 0.24849;
    public static final double DRIVE_KV = 0.119;
    public static final double DRIVE_KA = 0.0028462;

//    public static final double PATH_FOLLOW_DRIVE_KP = 1.0;

    public static final double PATH_FOLLOW_DRIVE_KP = 1.0;
    public static final double PATH_FOLLOW_DRIVE_KI = 0;
    public static final double PATH_FOLLOW_DRIVE_KD = 0;

    public static final double PATH_FOLLOW_STEER_KP = 3.0;
    public static final double PATH_FOLLOW_STEER_KI = 0;
    public static final double PATH_FOLLOW_STEER_KD = 0.0;

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  }

  public final class SHOOTER {

    public static final int RIGHT_MOTOR_CAN_ID = 19;
    public static final int LEFT_MOTOR_CAN_ID = 13;

    public static final Current SHOOTER_CURRENT_LIMIT = Amps.of(60);


    public static final Voltage SHOOTER_P = Volts.of(0.6);
    public static final Voltage SHOOTER_I = Volts.of(0);
    public static final Voltage SHOOTER_D = Volts.of(0);
    public static final Voltage SHOOTER_S = Volts.of(0.5);
    public static final Voltage SHOOTER_V = Volts.of(0.1037);
    public static final Voltage SHOOTER_A = Volts.of(0.2432);

    public static final Time SHOOTER_FILTER_TIME_CONSTANT = Seconds.of(0.01);

    //TODO: Change these values, these are default
    public static final Velocity<AngularAccelerationUnit> MAX_JERK = RotationsPerSecondPerSecond.per(Second).of(300);
    public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(100);

    // public static final Distance BACKWHEEL_DIAMETER = Inches.of(2.154);
    // public static final Distance BACKWHEEL_CIRCUMFERENCE = Inches.of(BACKWHEEL_DIAMETER.in(Inches) * Math.PI);
    // public static final LinearVelocity BACKWHEEL_DESIRED_VELOCITY = InchesPerSecond.of(FeetPerSecond.of(10d).in(FeetPerSecond) * 12);
    // public static final AngularVelocity BACKWHEEL_RPS_DESIRED = RevolutionsPerSecond.of((BACKWHEEL_DESIRED_VELOCITY).in(InchesPerSecond)/(BACKWHEEL_CIRCUMFERENCE.in(Inches)));


    public static final double SHOOTER_TOLERANCE = 50;


    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/SHOOTER/";
  }

  public final class TURRET {
    public static final int TURRET_MOTOR_CAN_ID = 20;
    public static final Current CURRENT_LIMIT = Amps.of(50);
    public static final Voltage TURRET_P = Volts.of(40d); // 12;
    public static final Voltage TURRET_I = Volts.of(0.01);
    public static final Voltage TURRET_D = Volts.of(0.8); // 0.4;
    public static final Voltage TURRET_S = Volts.of(0.6);
    public static final Voltage TURRET_V = Volts.of(0d);
    public static final Voltage TURRET_A = Volts.of(0d);
    public static final double TURRET_GT = 80d;
    public static final double TURRET_G1 = 21d;
    public static final double TURRET_G2 = 23d;
    public static final double TURRET_GM = 25d;
    public static final double TURRET_TOTAL = TURRET_G1 * TURRET_G2;
    public static final double DEGREES_TO_MOTOR_ROTATIONS = (TURRET_GT / TURRET_GM) / 360d;

    //TODO: Check if this works
    public static final Velocity<AngularAccelerationUnit> MAX_JERK = RotationsPerSecondPerSecond.per(Second).of(3000);
    public static final AngularAcceleration MAX_ACCELERATION = RotationsPerSecondPerSecond.of(300);
    public static final AngularVelocity CRUISE_VELOCITY = RotationsPerSecond.of(50); 

    //TODO: Update these constants
    public static final Angle E1_OFFSET = Degrees.of(286);
    public static final Angle E2_OFFSET = Degrees.of(323.4);
    //Try these out?
    public static final Angle MAX_ROTATION_LIMIT = Degrees.of(180);
    public static final Angle FORWARD_LIMIT = MAX_ROTATION_LIMIT; // Degrees
    public static final Angle REVERSE_LIMIT = MAX_ROTATION_LIMIT.unaryMinus(); // Degrees
    public static final Angle TURRET_TOLERANCE = Degrees.of(0.75); // Degrees
    public static final Angle CRT_TOLERANCE = Rotations.of(0.004);
    public static final Angle TURRET_ANGLE_OFFSET = Degrees.of(180); // The turret zero position is at 180
                                                          // degrees relative to the
                                                          // front of the robot

    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/TURRET/";
  }

  public static final class SCORING {
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/SCORING/";
    // targets
    public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.66, 4.04, new Rotation2d(0));
    public static final Pose2d BLUE_PASSING_LOW_POSE = new Pose2d(2.07, 2.02, new Rotation2d(0));
    public static final Pose2d BLUE_PASSING_HIGH_POSE = new Pose2d(2.07, 6.05, new Rotation2d(0));

    public static final Pose2d RED_HUB_POSE = new Pose2d(11.94, 4.04, new Rotation2d(0));
    public static final Pose2d RED_PASSING_LOW_POSE = new Pose2d(14.48, 2.02, new Rotation2d(0));
    public static final Pose2d RED_PASSING_HIGH_POSE = new Pose2d(14.48, 6.05, new Rotation2d(0));

    public static final double TAG_HUB_HEIGHT = 1.12395;
    public static final double SHOOTER_THRESHOLD = 500; // RPM

    public static final double SHORT_RANGE_SHOT = 2d;

    public static final double SHORT_RANGE_K = 2.3;
  }

  public static class INTAKE {
    // CAN ID for the Intake motor
    // TODO: Change these names to match the actual location of the motors
    public static final int INTAKE_MOTOR_LEFT_CAN_ID = 34;
    public static final int INTAKE_ROLLER_RIGHT_CAN_ID = 29;
    public static final int INTAKE_EXTEND_CAN_ID = 31;

    // Current limit for the Intake motor
    public static final int EXTEND_CURRENT_LIMIT = 25;
    public static final int ROLLER_CURRENT_LIMIT = 55;

    // PID tuning constants for the NEO Motors, these are initial and WILL change

    public static final double INTAKE_EXTEND_P = 0.5; //0.5
    public static final double INTAKE_EXTEND_I = 0;
    public static final double INTAKE_EXTEND_D = 0;
    public static final double EXTEND_ROTATIONS = 18.8;
    public static final double EXTEND_SOFT_LIMIT = 3;
    public static final double EXTEND_PROFILE_ERROR = 10;
    public static final double RETRACT_LIMIT = 0.5;
    public static final double RETRACT_VOLTAGE = -6;
    public static final double ROLLER_VOLTAGE = 11;
    public static final double ROLLER_VOLTAGE_SLOW = 7;

    // TODO: Tune and change these!
    public static final double CRUISE_VELOCITY = 2000;
    public static final double MAX_ACCELERATION = 4000;

    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Intake/";
  }

  public final class INDEXER {
    // CAN IDs for the Indexer motors
    public static final int SPINNER_CAN_ID = 16;
    public static final int OUTPUT_CAN_ID = 38;

    public static final double SPIN_P = 0.0001;
    public static final double SPIN_I = 0;
    public static final double SPIN_D = 0;
    public static final double SPIN_S = 0;


    // Current limits for the Indexer motors
    // TODO: tune these limits
    public static final int SPIN_CURRENT_LIMIT = 60;
    public static final int OUTPUT_CURRENT_LIMIT = 60;

    public static final double SPIN_MOTOR_SPEED = 5000;
    public static final double SPIN_MOTOR_STOP_SPEED = 0.0;
    public static final double OUTPUT_MOTOR_SPEED = 5000;
    public static final double OUTPUT_MOTOR_STOP_SPEED = 0.0;

    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Indexer/";
  }

  public final class RANGE_TABLE {
    public static final double RANGE_TABLE_STEP = 0.2; // meters
    public static final double MAX_TABLE_DISTANCE = 10.0; // meters
  }

  public final class LEDS {
    public static final Color PRISMARINE = new Color(126, 171, 172);
    public static final Color TEAL = new Color(0, 64, 192);
    public static final Color ORANGE = new Color(192, 64, 0);
    public static final Color WHITE = new Color(255, 255, 255);
    public static final Color GREEN = new Color(0, 255, 0);
    public static final Color RED = new Color(255, 0, 0);
    public static final Color OFF = new Color(0, 0, 0);
    public static final Color YELLOW = new Color(255, 255, 0);
    public static final Color PURPLE = new Color(255, 0, 255);
    public static final int LED_STRIP_LENGTH = 52;
    public static final int LED_CANDLE_COUNT = 8;
    public static final int FULL_LED_COUNT = LED_STRIP_LENGTH + LED_CANDLE_COUNT;
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/LEDS/";
  }

}
