package frc.robot;

import static edu.wpi.first.units.Units.Feet;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.units.measure.Distance;
import frc.robot.subsystems.swerve.TunerConstants;


public final class Constants {
  public final class SHARED {
    public static final String LOG_FOLDER = "CustomLogs";
}

public final class MEASUREMENTS {
    public static final Distance FIELD_LENGTH = Feet.of(27);
    public static final Distance FIELD_WIDTH = Feet.of(54);
}

public final class CONTROLLERS {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1;
}

public final class VISION {
    public static final String LOG_PATH = SHARED.LOG_FOLDER+"ScoreCoral";
    public static final double MAX_ACCEPTABLE_AMBIGUITY = 0.1;
    public static final double REJECT_SINGLE_TAG_POSE_ESTIMATE_RANGE = 1.4d;
    public static final int POSE_AVERAGER_VALUE = 50;
}
public final class SWERVE {
    public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Swerve/";
    public static final double SNAP_TO_kP = 3.7;
    public static final double SNAP_TO_kI = 0.0;
    public static final double SNAP_TO_kD = 0.1;
    public static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND =MetersPerSecond.of(4.73).baseUnitMagnitude();
    public static final TrajectoryConfig PATH_FOLLOW_TRAJECTORY_CONFIG = new TrajectoryConfig(4.5, 3);
    public static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = RadiansPerSecond.of(720).baseUnitMagnitude();
    public static final double TRANSLATE_POWER_FAST = 1.0; 
    public static final double ROTATE_POWER_FAST = 0.5; 
    public static final double TRANSLATE_POWER_SLOW = 0.5;
    public static final double ROTATE_POWER_SLOW = 0.3;
    public static final int TRANSLATION_SMOOTHING_AMOUNT = 3;
    public static final int ROTATION_SMOOTHING_AMOUNT = 1;
    public static final double JOYSTICK_EXPONENT = 1.75;

    public static final double PATH_FOLLOW_TRANSLATE_kP = 1.5; 
    public static final double PATH_FOLLOW_TRANSLATE_kI = 0.0;
    public static final double PATH_FOLLOW_TRANSLATE_kD = 0.15;

    //TODO: Double check that these still work
    public static final double PATH_FOLLOW_ROTATE_kP = 2.0;
    public static final double PATH_FOLLOW_ROTATE_kI = 0.0;
    public static final double PATH_FOLLOW_ROTATE_kD = 0.0;

    public static final double PATH_FOLLOW_ROTATE_MAX_VELOCITY = 1.5 * Math.PI;
    public static final double PATH_FOLLOW_ROTATE_MAX_ACCELLERATION = 1.5 * Math.PI;

    public static final double PATH_FOLLOW_TRANSLATE_POSITION_TOLERANCE = 0.03; // Meters
    public static final double PATH_FOLLOW_TRANSLATE_VELOCITY_TOLERANCE = 0.05;

    public static final double PATH_FOLLOW_ROTATE_POSITION_TOLERANCE = 0.04; // Radians
    public static final double PATH_FOLLOW_ROTATE_VELOCITY_TOLERANCE = 0.02;

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
}
    
public class SUPPLIERS{
    public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Suppliers/";
}

public final class SHOOTER {
  //Left and right classifications are for looking from the robots viewpoint
    public static final int RIGHT_SHOOTER_MOTOR = 38;
    public static final int LEFT_SHOOTER_MOTOR = 36;
    //PID tuning constants for the NEO Motors, these are initial and WILL change
    public static final double MOTOR_P = 0.01;
    public static final double MOTOR_I = 0;
    public static final double MOTOR_D = 0;
    //These are to use motion magic for the shooter with Kraken X60's, easier than PID tuning
    public static final double MAX_ACCELERATION = 0;
    //For velocity control, cruise velocity is not needed, but it might throw errors if not included so this is here as a placeholder
    public static final double CRUISE_VELOCITY = 0;
    public static final double SHOOTER_HEIGHT = 0;
    public static final double HUB_HEIGHT = 0;
}

public static class INTAKE{
    // CAN ID for the Intake motor
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
//Left and right classifications are for looking from the robots viewpoint
  public static final int INDEXER_SPINNER_CAN_ID = 31;
  public static final int INDEXER_OUTPUT_CAN_ID = 35; 
  //PID tuning constants for the NEO Motors, these are initial and WILL change
  public static final double INDEXER_SPINNER_P = 0.01;
  public static final double INDEXER_SPINNER_I = 0;
  public static final double INDEXER_SPINNER_D = 0;
   public static final double INDEXER_SPINNER_VELOCITY = 4000;

  public static final double INDEXER_OUTPUT_P = 0.0001;
  public static final double INDEXER_OUTPUT_I = 0;
  public static final double INDEXER_OUTPUT_D = 0;
  public static final double INDEXER_OUTPUT_VELOCITY = 5000;
  //These are to use motion magic for the shooter with Kraken X60's, easier than PID tuning
  public static final double MAX_ACCELERATION = 0;
}

  public static final class SCORING{
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Scoring/";
  }
}