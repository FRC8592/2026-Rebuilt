// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public final class SHARED {
    public static final String LOG_FOLDER = "CustomLogs";
  }

  public final class MEASUREMENTS {
    public static final double FIELD_LENGTH_METERS = 16.4592;
    public static final double FIELD_WIDTH_METERS = 8.2296;
  }

  public static class CONTROLLERS  {
    public static final int DRIVER_PORT = 0;
  }

  public final class ROBOT {
    public static final String LOG_PATH = SHARED.LOG_FOLDER + "/Robot/";
  }

  public class SUPPLIERS{
      public static final String LOG_PATH = SHARED.LOG_FOLDER+"/Suppliers/";
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
        public static final double ROTATE_POWER_FAST = 0.75; 
        public static final double TRANSLATE_POWER_SLOW = 0.5;
        public static final double ROTATE_POWER_SLOW = 0.3;

        public static final int TRANSLATION_SMOOTHING_AMOUNT = 3;
        public static final int ROTATION_SMOOTHING_AMOUNT = 1;

        public static final double JOYSTICK_EXPONENT = 1.2;

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
    }
}
