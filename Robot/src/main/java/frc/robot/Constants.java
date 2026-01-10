package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.subsystems.swerve.TunerConstants;


public final class Constants {

  

  public final class SHARED {
    public static final String LOG_FOLDER = "CustomLogs";
}

public final class MEASUREMENTS {
    public static final double FIELD_LENGTH_METERS = 27 * CONVERSIONS.FEET_TO_METERS;
    public static final double FIELD_WIDTH_METERS = 54 * CONVERSIONS.FEET_TO_METERS;
    
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

public final class CAN {
    public static final int PDH_CAN_ID = 1;
    public static final int BOTTOM_LAUNCHER_MOTOR = 2; 
    public static final int TOP_LAUNCHER_MOTOR = 21; 
    public static final int INDEXER_MOTOR1_CAN_ID = 40;
    public static final int INDEXER_MOTOR2_CAN_ID = 41;
    public static final int INDEXER_MOTOR3_CAN_ID = 42;
    public static final int INDEXER_MOTOR4_CAN_ID = 43;
    
    public static final int INTAKE_MOTOR_SIDE_CAN_ID = 36;
    public static final int INTAKE_MOTOR_BOTTOM_CAN_ID = 38;
    public static final int PIVOT_INTAKE_MOTOR_CAN_ID = 44;
}

public final class VISION {
    public static final String LOG_PATH = SHARED.LOG_FOLDER+"ScoreCoral";
    public static final double OFFSET_DEPTH = 0.40; // Drivers requested for the robot to be as close to the april tag as possible
    public static final double OFFSET_LEFT_METERS = -0.137;
    public static final double OFFSET_RIGHT_METERS = 0.213; 
    public static final double ROT_OFFSET = 0d;
    public static final double SPEED_SCALE = 1.0;
    public static final double SPEED_MAX = 0.2; // originally 0.65

    public static final int MAX_LOCK_LOSS_TICKS = 20;

    public static final Transform3d CAMERA_OFFSETS = (
        new Transform3d(new Translation3d(0.17145, 0.20955, 0.2286), new Rotation3d(0, Math.toRadians(-13), Math.toRadians(-3)))
    );


    public static final String CAMERA_NAME = (
        "Arducam_OV9782_B" 
    );
    public static final double MAX_ACCEPTABLE_AMBIGUITY = 0.1;
    public static final double REJECT_SINGLE_TAG_POSE_ESTIMATE_RANGE = 1.4d;
    public static final int POSE_AVERAGER_VALUE = 50;
}
    
public final class SWERVE {
    public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Swerve/";

    //TODO: Double check that these PID constants still work
    public static final double SNAP_TO_kP = 3.7;
    public static final double SNAP_TO_kI = 0.0;
    public static final double SNAP_TO_kD = 0.1;

    public static final double MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND = 4.73;
    public static final TrajectoryConfig PATH_FOLLOW_TRAJECTORY_CONFIG = new TrajectoryConfig(4.5, 3);
    public static final double MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND = Math.toRadians(720);

    public static final boolean INVERT_LEFT_SIDE = false;
    public static final boolean INVERT_RIGHT_SIDE = true;

    public static final double SIMULATED_STEER_INERTIA = 0.00001;
    public static final double SIMULATED_DRIVE_INERTIA = 0.06;
    public static final double SIMULATION_LOOP_PERIOD = 0.005;
    public static final double STEER_FRICTION_VOLTAGE = 0.25;
    public static final double DRIVE_FRICTION_VOLTAGE = 0.25;

    //TODO: Tone these down appropriately as per BB rules
    public static final double TRANSLATE_POWER_FAST = 1.0; 
    public static final double ROTATE_POWER_FAST = 0.25; 
    public static final double TRANSLATE_POWER_SLOW = 0.5;
    public static final double ROTATE_POWER_SLOW = 0.3;

    public static final int TRANSLATION_SMOOTHING_AMOUNT = 3;
    public static final int ROTATION_SMOOTHING_AMOUNT = 1;

    public static final double JOYSTICK_EXPONENT = 1.75;

    public static final Rotation2d BLUE_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(0);
    public static final Rotation2d RED_PERSPECTIVE_ROTATION = Rotation2d.fromDegrees(180);

    public static final double PATH_FOLLOW_TRANSLATE_kP = 8d; // Was 8 in the last test
    public static final double PATH_FOLLOW_TRANSLATE_kI = 0d;
    public static final double PATH_FOLLOW_TRANSLATE_kD = 0d;

    //TODO: Double check that these still work
    public static final double PATH_FOLLOW_ROTATE_kP = 12;
    public static final double PATH_FOLLOW_ROTATE_kI = 0d;
    public static final double PATH_FOLLOW_ROTATE_kD = 0;

    public static final double PATH_FOLLOW_ROTATE_MAX_VELOCITY = 4 * Math.PI;
    public static final double PATH_FOLLOW_ROTATE_MAX_ACCELLERATION = 4 * Math.PI;

    public static final double PATH_FOLLOW_TRANSLATE_POSITION_TOLERANCE = 0.01; // Meters
    public static final double PATH_FOLLOW_TRANSLATE_VELOCITY_TOLERANCE = 0.02;

    public static final double PATH_FOLLOW_ROTATE_POSITION_TOLERANCE = 0.05; // Radians
    public static final double PATH_FOLLOW_ROTATE_VELOCITY_TOLERANCE = 0.03;

    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    public static final double MAX_ANGULAR_RATE = RotationsPerSecond.of(0.75).in(RadiansPerSecond);
}

public final class ROBOT {
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Robot/";
}

public class SUPPLIERS{
    public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Suppliers/";
}

public final class INDEXER {
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Indexer/";
    public static final int INDEXER_BEAM_BREAK_THRESHOLD_MM = 20; //subject to change

    public static final int INDEXER_BEAM_BREAK_1_PORT = 0;
    public static final int INDEXER_BEAM_BREAK_2_PORT = 1;
    public static final int INDEXER_BEAM_BREAK_3_PORT = 3;
}

public final class LAUNCHER {
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Launcher/";
}

public static class INTAKE{
    //DEFAULT VALUES, NEED TO CHANGE THESE LATER
    //these are for the PID of the Pivot Intake Motor
    public static final double INTAKE_POSITION_P = 5;
    public static final double INTAKE_POSITION_I = 0;
    public static final double INTAKE_POSITION_D = 0.1;
    //This is the gearbox that the motor will be plugged into
    public static final int GEARBOX_PIVOT_RATIO = 16;
    //this is to convert motor rotations to intake degrees
    public static final int INTAKE_DEGREES_TO_MOTOR_ROTATIONS = GEARBOX_PIVOT_RATIO/360;
    //This is to make sure that it stays within position, will need to change later
    public static final double WITHIN_POSITION_LIMITS = 5;
    //Set current limits of the Kraken so they do not overheat, default value of 40 for current limits for now
    public static final int INTAKE_CURRENT_LIMIT = 80;
    public static final int PIVOT_INTAKE_CURRENT_LIMIT = 40;
    //Set max velocity and max acceleration of Intake motor of Intake for motion magic set up
    public static final double PIVOT_INTAKE_MAX_VELOCITY = 2;
    public static final double PIVOT_INTAKE_MAX_ACCELERATION = 4;
    public static final double PIVOT_INTAKE_TOLERANCE = 0;
    //This is the position of the intake when we want to eject additional lunites
    public static final double EJECT_LUNITE_POSITION = 0;
    //This is the position of the intake when it is touching the ground
    public static final double SET_PIVOT_INTAKE = 0;
    //This should be 0 as the absolute encoder is intialized at 0 of stow, but maybe not for match setup so leave this here
    public static final double STOW_PIVOT_INTAKE = 0;
    //Amount of motor rotations to eject
    public static final int EJECT_MOTOR_ROTATIONS = 1000;
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Intake/";

  }

  public static final class SCORING{
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/SCORING/";
  }
}
