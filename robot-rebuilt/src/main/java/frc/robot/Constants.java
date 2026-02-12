package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.Shooter;


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
      new Transform3d(new Translation3d(0.51, -0.31, 0.27), new Rotation3d(0, Math.toRadians(-15), 0))
    );

    public static final String CAMERA_NAME = (
        "Arducam_OV9782_B" 
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

    //TODO: Tone these down appropriately as per BB rules
    public static final double TRANSLATE_POWER_FAST = 1.0; 
    public static final double ROTATE_POWER_FAST = 0.5; 
    public static final double TRANSLATE_POWER_SLOW = 0.5;
    public static final double ROTATE_POWER_SLOW = 0.3;

    public static final int TRANSLATION_SMOOTHING_AMOUNT = 3;
    public static final int ROTATION_SMOOTHING_AMOUNT = 1;

    public static final double JOYSTICK_EXPONENT = 1.75;

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
  }

public final class SHOOTER {
  //Left and right classifications are for looking from the robots viewpoint
    public static final int RIGHT_SHOOTER_MOTOR = 10;
    public static final int LEFT_SHOOTER_MOTOR = 11;
    //PID tuning constants for the NEO Motors, these are initial and WILL change
    public static final double MOTOR_P = 0.01;
    public static final double MOTOR_I = 0;
    public static final double MOTOR_D = 0;
    //These are to use motion magic for the shooter with Kraken X60's, easier than PID tuning
    public static final double MAX_ACCELERATION = 0;
    public static final double CRUISE_VELOCITY = 0;
    public static final double SHOOTER_HEIGHT = 0;
    public static final double HUB_HEIGHT = 0;
  }

  public final class TURRET{
    public static final int TURRET_MOTOR = 19;
    public static final double TURRET_P = 5;
    public static final double TURRET_I = 3;
    public static final double TURRET_D = 0.45;
    public static final double TURRET_V = 0.7;
    public static final int TURRET_TG = 96;
    public static final int TURRET_G1 = 10;
    public static final int TURRET_G2 = 11;
    public static final int TURRET_TOTAL = TURRET_G1 * TURRET_G2;
    public static final double DEGREES_TO_MOTOR_ROTATIONS = (96.0/10)/360;
    public static final int INITIAL_MAX_ACCELERATION = 60;
    public static final int INITIAL_CRUISE_VELOCITY = 6;
    public static final double E1_OFFSET = 340.19;
    public static final double E2_OFFSET = 189.6;

    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/TURRET/";
}


  public static final class SCORING{
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/SCORING/";

    public static final double HUB_X = 4.02844;
    public static final double HUB_Y = 4.445;
  }



public final class INTAKE{
    public static final int INTAKE_MOTOR_CAN_ID = 44;

    // Current limit for the Intake motor
    public static final int INTAKE_CURRENT_LIMIT_STALL = 80;
    public static final int INTAKE_CURRENT_LIMIT_FREE = 80;

    //PID tuning constants for the NEO Motors, these are initial and WILL change
    public static final double INTAKE_P = 0.01;
    public static final double INTAKE_I = 0;
    public static final double INTAKE_D = 0;
    public static final double INTAKE_VI = 3000;
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Intake/";

  }

  public final class INDEXER {
    // CAN IDs for the Indexer motors
    public static final int SPINNER_CAN_ID = 31;
    public static final int OUTPUT_CAN_ID = 35;

    // Current limts for the Indexer motors
    public static final int SPINNER_CURRENT_LIMIT_STALL = 80;
    public static final int SPINNER_CURRENT_LIMIT_FREE = 80;
    public static final int OUTPUT_CURRENT_LIMIT_STALL = 80;
    public static final int OUTPUT_CURRENT_LIMIT_FREE = 80;

    //PID tuning constants for the NEO Motors, these are initial and WILL change
    public static final double SPINNER_P = 0.00045;
    public static final double SPINNER_I = 0;
    public static final double SPINNER_D = 0.003;
    public static final double SPINNER_VI = 5000;

    public static final double OUTPUT_P = 0.01;
    public static final double OUTPUT_I = 0;
    public static final double OUTPUT_D = 0;
    public static final double OUTPUT_VI = 5000;
      
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Indexer/";
  }

  // public static final class SCORING{
  //   public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Scoring/";
  // }
}