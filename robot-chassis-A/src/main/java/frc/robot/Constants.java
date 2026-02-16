package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;
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

    public static final Transform3d CAMERA_OFFSETS = (
        new Transform3d(new Translation3d(0.14, 0.325, 0.22), new Rotation3d(0, Math.toRadians(-36), 0))
    );

    public static final String CAMERA_NAME = (
        "Arducam_OV9782_H" 
    );

    public static final double MAX_ACCEPTABLE_AMBIGUITY = 0.1;
    public static final double REJECT_SINGLE_TAG_POSE_ESTIMATE_RANGE = 1.4d;
    public static final int POSE_AVERAGER_VALUE = 50;
  }

  public final class SWERVE {
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Swerve/";

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
    public static final double ROTATE_POWER_FAST = 1.0; 
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

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  }

public final class SHOOTER {
  //Left and right classifications are for looking from the robots viewpoint
    public static final int BACKWHEEL_MOTOR_CAN_ID = 19;
    public static final int FLYWHEEL_MOTOR_CAN_ID = 13;
    //PID tuning constants for the NEO Motors, these are initial and WILL change
    public static final double FLYWHEEL_P = 0.01;
    public static final double FLYWHEEL_I = 0;
    public static final double FLYWHEEL_D = 0;
    public static final double FLYWHEEL_V = 0;
    public static final double BACKWHEEL_P = 0.01;
    public static final double BACKWHEEL_I = 0;
    public static final double BACKWHEEL_D = 0;
    public static final double BACKWHEEL_V = 0;

    public static final double FLYWHEEL_VI = 0;

    public static final double SHOOTER_HEIGHT = 0;
    public static final double HUB_HEIGHT = 0;

    public static final double FLYWHEEL_DIAMETER_INCHES = 4;
    public static final double BACKWHEEL_DIAMETER_INCHES = 3; //change later
  }

  public final class TURRET{
    public static final int TURRET_MOTOR = 20;
    public static final double TURRET_P = 4;
    public static final double TURRET_I = 3;
    public static final double TURRET_D = 0.2;
    public static final double TURRET_V = 0.7;
    public static final int TURRET_TG = 96;
    public static final int TURRET_G1 = 10;
    public static final int TURRET_G2 = 11;
    public static final int TURRET_TOTAL = TURRET_G1 * TURRET_G2;
    public static final double DEGREES_TO_MOTOR_ROTATIONS = (96.0/10)/360;
    public static final int INITIAL_MAX_ACCELERATION = 60;
    public static final int INITIAL_CRUISE_VELOCITY = 6;
    public static final double E1_OFFSET = 286;
    public static final double E2_OFFSET = 323.4;

    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/TURRET/";
  }

  public static final class SCORING{
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/SCORING/";
  }

  public static class INTAKE{
    // CAN ID for the Intake motor
    public static final int INTAKE_MOTOR_CAN_ID = 44;
    public static final int INTAKE_ROLLER_LEFT_CAN_ID = 0; 
    public static final int INTAKE_ROLLER_RIGHT_CAN_ID = 29;
    public static final int INTAKE_EXTEND_CAN_ID = 31; 

    // Current limit for the Intake motor
    public static final int INTAKE_CURRENT_LIMIT_STALL = 80;
    public static final int INTAKE_CURRENT_LIMIT_FREE = 0; 
    
    public static final int EXTEND_CURRENT_LIMIT_STALL = 0; 
    public static final int EXTEND_CURRENT_LIMIT_FREE = 0; 
    //PID tuning constants for the NEO Motors, these are initial and WILL change
    public static final double INTAKE_LEFT_P = 0.01;
    public static final double INTAKE_LEFT_I = 0;
    public static final double INTAKE_LEFT_D = 0;
    public static final double INTAKE_LEFT_VI = 3000;

    public static final double INTAKE_RIGHT_P = 0.01;
    public static final double INTAKE_RIGHT_I = 0;
    public static final double INTAKE_RIGHT_D = 0;
    public static final double INTAKE_RIGHT_VI = 3000;

    public static final double INTAKE_EXTEND_P = 0.01;
    public static final double INTAKE_EXTEND_I = 0;
    public static final double INTAKE_EXTEND_D = 0;
    public static final double INTAKE_EXTEND_POS = 3000;

    public static final double EXTEND_ROTATIONS = 2; 
    public static final double DESIRED_ROTATIONS_EXTEND = 0; 

    public static final double EXTEND_TORQUE_CURRENT = 0; 
    
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Intake/";
  }

  public final class INDEXER {
    // CAN IDs for the Indexer motors
    public static final int SPINNER_CAN_ID = 16;
    public static final int OUTPUT_CAN_ID = 30;

    public static final double SPIN_P = 0;
    public static final double SPIN_I = 0;
    public static final double SPIN_D = 0;

    public static final double OUTPUT_P = 0;
    public static final double OUTPUT_I = 0;
    public static final double OUTPUT_D = 0;

    // Current limts for the Indexer motors
    public static final int SPIN_CURRENT_LIMIT_STALL = 80;
    public static final int SPIN_CURRENT_LIMIT_FREE = 80;
    public static final int OUTPUT_CURRENT_LIMIT_STALL = 80;

    public static final double SPIN_MOTOR_SPEED = 1.0;
    public static final double SPIN_MOTOR_STOP_SPEED = 0.0;
    public static final double OUTPUT_MOTOR_SPEED = 1.0;
    public static final double OUTPUT_MOTOR_STOP_SPEED = 0.0;
    
    public static final double MOTOR_MAX_RPS = 6380 / 60;

    public static final double SPIN_MOTOR_RPS = SPIN_MOTOR_SPEED * MOTOR_MAX_RPS;
    public static final double OUTPUT_MOTOR_RPS = SPIN_MOTOR_SPEED * MOTOR_MAX_RPS;
      
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Indexer/";

  }

  public final class RANGE_TABLE {
    public static final double RANGE_TABLE_STEP = 0.2; //meters
    public static final double MAX_TABLE_DISTANCE = 10.0; //meters 
  }

}
