package frc.robot;

import static edu.wpi.first.units.Units.*;

import frc.robot.subsystems.swerve.TunerConstants;

/**
 * Values computed from other values. The .conf files have no arithmetic, so anything derived
 * from config values (or from other code, like TunerConstants) lives here. Everything else,
 * including units, poses and colors, is generated into {@link Constants} from src/main/deploy/*.conf.
 */
public final class CodeConstants {
  public final class SWERVE {
    public static final double MAX_SPEED = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
  }

  public final class TURRET {
    public static final double TURRET_TOTAL = Constants.TURRET.TURRET_G1 * Constants.TURRET.TURRET_G2;
    public static final double DEGREES_TO_MOTOR_ROTATIONS =
        (Constants.TURRET.TURRET_GT / Constants.TURRET.TURRET_GM) / 360d;
  }

  public final class LEDS {
    // Total number of LEDs including the CANdle LEDs
    public static final int FULL_LED_COUNT = Constants.LEDS.LED_STRIP_LENGTH + Constants.LEDS.LED_CANDLE_COUNT;
  }
}
