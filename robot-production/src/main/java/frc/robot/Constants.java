package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
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

    //camera G
    public static final Transform3d CAMERA_OFFSETS_RIGHT = (
        new Transform3d(new Translation3d(-0.334588, -0.072009, 0.5212334), new Rotation3d(0, Math.toRadians(-30), Math.toRadians(270)))
    );

    //camera F
    public static final Transform3d CAMERA_OFFSETS_BACK = (
        new Transform3d(new Translation3d(0.2797302, -0.0772414,-0.5218176), new Rotation3d(0,Math.toRadians(-30),Math.toRadians(180)))
    ); 

    //camera H 
    public static final Transform3d CAMERA_OFFSETS_LEFT = (
        new Transform3d(new Translation3d(0.334588, -0.072009, 0.5212334), new Rotation3d(0, Math.toRadians(-30), Math.toRadians(90)))
    ); 

    public static final String CAMERA_NAME_RIGHT = (
        "Right_Arducam_OV9782_G" 
    );

    public static final String CAMERA_NAME_BACK = (
        "Back_Arducam_OV9782_F"
    ); 

    public static final String CAMERA_NAME_LEFT = (
      "Left_Arducam_OV9782_H"
    ); 
    public static final double MAX_ACCEPTABLE_AMBIGUITY = 0.1;
    public static final double REJECT_SINGLE_TAG_POSE_ESTIMATE_RANGE = 3.5;
  }

  public final class SWERVE {
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/SWERVE/";

    //TODO: Double check that these PID constants still work
    public static final double SNAP_TO_kP = 3.7;
    public static final double SNAP_TO_kI = 0.0;
    public static final double SNAP_TO_kD = 0.1;

    public static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = 4.73;
    public static final TrajectoryConfig PATH_FOLLOW_TRAJECTORY_CONFIG = new TrajectoryConfig(4.5, 3);
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

    //swerve pid constants below
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
  //Left and right classifications are for looking from the robots viewpoint
    public static final int BACKWHEEL_MOTOR_CAN_ID = 19;
    public static final int FLYWHEEL_MOTOR_CAN_ID = 13;

    public static final double FLYWHEEL_CURRENT_LIMIT = 120.0;
    public static final double BACKWHEEL_CURRENT_LIMIT = 40.0;
    //PID tuning constants for the NEO Motors, these are initial and WILL change
    public static final double FLYWHEEL_P = 7.0;
    public static final double FLYWHEEL_I = 0.0;
    public static final double FLYWHEEL_D = 0.0;
    public static final double FLYWHEEL_S = 0.3;
    public static final double FLYWHEEL_V = 0.1054;
    
    public static final double BACKWHEEL_P = 3.0;
    public static final double BACKWHEEL_I = 0.0;
    public static final double BACKWHEEL_D = 0.0;
    public static final double BACKWHEEL_S = 0.4;
    public static final double BACKWHEEL_V = 0.125;

    public static final double FLYWHEEL_VI = 3200;

    public static final double BACKWHEEL_VELOCITY = 1146;

    public static final double SHOOTER_HEIGHT = 0;
    public static final double HUB_HEIGHT = 0;

    public static final double SHOOTER_TOLERANCE = 50;

    public static final double FLYWHEEL_DIAMETER_INCHES = 4;
    public static final double BACKWHEEL_DIAMETER_INCHES = 2; // TODO: Update if wheel diameter changes

    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/SHOOTER/";
  }

  public final class TURRET{
    public static final int TURRET_MOTOR_CAN_ID = 20;
    public static final int TURRET_CURRENT_LIMIT = 200;
    public static final double TURRET_P0 = 40; //12;
    public static final double TURRET_I0 = 0.01;
    public static final double TURRET_D0 = 0.8; //0.4;
    public static final double TURRET_S = 0.6;
    public static final double TURRET_P1 = 1;
    public static final double TURRET_I1 = 0.0;
    public static final double TURRET_D1 = 0.0;

    public static final int TURRET_TG = 80;
    public static final int TURRET_G1 = 19;
    public static final int TURRET_G2 = 23;
    public static final int TURRET_TOTAL = TURRET_G1 * TURRET_G2;
    public static final double DEGREES_TO_MOTOR_ROTATIONS = (80.0 / 25.0) / 360;
    public static final double MAX_JERK = 3000;
    public static final int MAX_ACCELERATION = 300;
    public static final int CRUISE_VELOCITY = 50;
    public static final double E1_OFFSET = 286;
    public static final double E2_OFFSET = 323.4;
    public static final double FORWARD_LIMIT = 160; // Degrees
    public static final double REVERSE_LIMIT = -160; // Degrees
    public static final double TURRET_TOLERANCE = 8; // Degrees
    public static final double TURRET_ANGLE_OFFSET = 180; // The turret zero position is at 180 degrees relative to the front of the robot 

    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/TURRET/";
  }

  public static final class SCORING{
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/SCORING/";
    //targets
    public static final Pose2d BLUE_HUB_POSE = new Pose2d(4.66, 4.04, new Rotation2d(0));
    public static final Pose2d BLUE_PASSING_LOW_POSE = new Pose2d(2.07, 2.02, new Rotation2d(0));
    public static final Pose2d BLUE_PASSING_HIGH_POSE = new Pose2d(2.07, 6.05, new Rotation2d(0));
    
    public static final Pose2d RED_HUB_POSE = new Pose2d(11.94, 4.04, new Rotation2d(0));
    public static final Pose2d RED_PASSING_LOW_POSE = new Pose2d(14.48, 2.02, new Rotation2d(0));
    public static final Pose2d RED_PASSING_HIGH_POSE = new Pose2d(14.48, 6.05, new Rotation2d(0));

    public static final double SHOOTER_THRESHOLD = 500; //RPM
  }

  public static class INTAKE{
    // CAN ID for the Intake motor
    //TODO: Change these names to match the actual location of the motors
    public static final int INTAKE_MOTOR_LEFT_CAN_ID = 34;
    public static final int INTAKE_ROLLER_RIGHT_CAN_ID = 29;
    public static final int INTAKE_EXTEND_CAN_ID = 31; 

    // Current limit for the Intake motor
    public static final int EXTEND_CURRENT_LIMIT = 50;
    public static final int ROLLER_CURRENT_LIMIT = 50; 

    //PID tuning constants for the NEO Motors, these are initial and WILL change
    public static final double INTAKE_LEFT_P = 0.01;
    public static final double INTAKE_LEFT_I = 0;
    public static final double INTAKE_LEFT_D = 0;
    public static final double INTAKE_LEFT_VI = 10000;

    public static final double INTAKE_RIGHT_P = 1;
    public static final double INTAKE_RIGHT_I = 0;
    public static final double INTAKE_RIGHT_D = 0;
    public static final double INTAKE_RIGHT_VI = 6000;

    public static final double INTAKE_EXTEND_P = 0.5;
    public static final double INTAKE_EXTEND_I = 0;
    public static final double INTAKE_EXTEND_D = 0;
    public static final double INTAKE_EXTEND_POS = 3000;

    public static final double EXTEND_ROTATIONS = 8; 
    public static final double DESIRED_ROTATIONS_EXTEND = 3.4; //3.645
    public static final double EXTEND_CRUISE_RPM = 250.0;
    public static final double EXTEND_MAX_ACCEL_RPM_PER_SEC = 200;
    public static final double EXTEND_ALLOWED_ERROR_ROT = 0.05;

    public static double RETRACT_ROTATION_INCREMENT = 0.005; 

    //TODO: Tune and change these!
    public static final double CRUISE_VELOCITY = 1000;
    public static final double MAX_ACCELERATION = 1000;

    
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

    public static final double OUTPUT_P = 0.0001;
    public static final double OUTPUT_I = 0;
    public static final double OUTPUT_D = 0;

    // Current limits for the Indexer motors
    //TODO: tune these limits
    public static final int SPIN_CURRENT_LIMIT = 60;
    public static final int OUTPUT_CURRENT_LIMIT = 60;

    public static final double SPIN_MOTOR_SPEED = 5000;
    public static final double SPIN_MOTOR_STOP_SPEED = 0.0;
    public static final double OUTPUT_MOTOR_SPEED = 5000;
    public static final double OUTPUT_MOTOR_STOP_SPEED = 0.0;

    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Indexer/";
  }

  public final class RANGE_TABLE {
    public static final double RANGE_TABLE_STEP = 0.2; //meters
    public static final double MAX_TABLE_DISTANCE = 10.0; //meters 
  }

  public final class LEDS{
    public static final Color PRISMARINE = new Color(126,171,172); 
    public static final Color TEAL = new Color(0, 64, 192);
    public static final Color ORANGE = new Color(192, 64, 0);
    public static final Color WHITE = new Color(255, 255, 255);
    public static final Color GREEN = new Color(0,255,0);
    public static final Color RED = new Color(255, 0, 0);
    public static final Color OFF = new Color(0, 0, 0);
    public static final Color YELLOW = new Color(255,255,0);
    public static final Color PURPLE = new Color(255,0,255);
    public static final int LED_STRIP_LENGTH = 52;
    public static final int LED_CANDLE_COUNT= 8;
    public static final int FULL_LED_COUNT = LED_STRIP_LENGTH+LED_CANDLE_COUNT;
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/LEDS/";
    }


}
