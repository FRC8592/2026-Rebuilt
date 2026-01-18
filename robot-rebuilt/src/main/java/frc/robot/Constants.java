// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.util.Color;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class CONTROLLERS {
    public static final int DRIVER_PORT = 0;
    public static final int OPERATOR_PORT = 1; 
  }

  public final class CAN {
    public static final int INTAKE_MOTOR_CAN_ID = 44;
  }

  public final class INTAKE {
    public static final int GEARBOX_PIVOT_RATIO = 13;
     public static final int INTAKE_DEGREES_TO_MOTOR_ROTATIONS = GEARBOX_PIVOT_RATIO/360;
  }

      public final class LEDS{
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
    }
}
