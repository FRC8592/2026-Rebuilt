
package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import com.ctre.phoenix6.controls.RainbowAnimation;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDS;

// 1. Spindexer Jamming  2. Shooting 3. Running Indexer 

public class LEDs extends SubsystemBase {
        // TODO: RESTRICTED WHEELS, INTAKE SUCCESSFULLY LEDS, PROGRESS BAR FOR THE
        private CANdle candle;
        private Timer timer = new Timer();
        private boolean useRainbow = false;
        private static boolean canShoot = false;
        private boolean trackingTarget = false;
        private boolean spindexerStuck = false; 

        private int hasTags;
        private final RainbowAnimation rainbow = new RainbowAnimation(1, 26);
        
                public LEDs() {
                        CANdleConfiguration configAll = new CANdleConfiguration();
                        configAll.LED = new LEDConfigs().withBrightnessScalar(1)
                                        .withStripType(StripTypeValue.GRB)
                                        .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning);
                        configAll.CANdleFeatures = new CANdleFeaturesConfigs()
                                        .withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled)
                                        .withVBatOutputMode(VBatOutputModeValue.Modulated);
                        candle = new CANdle(33);
                        candle.getConfigurator().apply(configAll);
                        timer.start();
                }
        
                // SETTERS 
                public void setCanShoot() { //(3)
                        candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, LEDS.LED_HALF_STRIP_LENGTH)
                                        .withColor(new RGBWColor((int) (LEDS.TEAL.red * 255),
                                                        (int) (LEDS.TEAL.green * 255),
                                                        (int) (LEDS.TEAL.blue * 255))));
                }
        
                public void setCannotShoot() { //(2)
                        candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, LEDS.LED_HALF_STRIP_LENGTH)
                                        .withColor(new RGBWColor((int) (LEDS.ORANGE.red * 255),
                                                        (int) (LEDS.ORANGE.green * 255),
                                                        (int) (LEDS.ORANGE.blue * 255))));
                }
        
                public void setOff() {
                        candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT)
                                        .withColor(new RGBWColor((int) (LEDS.OFF.red * 255),
                                                        (int) (LEDS.OFF.green * 255),
                                                        (int) (LEDS.OFF.blue * 255))));
                }
        
                public void displayindexerRunning() { //(4)
                        candle.setControl(new SolidColor(LEDS.LED_HALF_STRIP_LENGTH, LEDS.FULL_LED_COUNT)
                                        .withColor(new RGBWColor((int) (LEDS.WHITE.red * 255),
                                                        (int) (LEDS.WHITE.green * 255),
                                                        (int) (LEDS.WHITE.blue * 255))));
                }
        
                public void displayHasTagsLEDs() {
                        if (hasTags >= 2) {
                                candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT)
                                                .withColor(new RGBWColor((int) (LEDS.GREEN.red * 255),
                                                                (int) (LEDS.GREEN.green * 255),
                                                                (int) (LEDS.GREEN.blue * 255))));
                        } else if (hasTags == 1) {
                                candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT)
                                                .withColor(new RGBWColor((int) (LEDS.YELLOW.red * 255),
                                                                (int) (LEDS.YELLOW.green * 255),
                                                                (int) (LEDS.YELLOW.blue * 255))));
                        } else {
                                candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT)
                                                .withColor(new RGBWColor((int) (LEDS.RED.red * 255),
                                                                (int) (LEDS.RED.green * 255),
                                                                (int) (LEDS.RED.blue * 255))));
                        }
                }
        
                public void setHasTags(int newHasTags) {
                        hasTags = newHasTags;
                        Logger.recordOutput(LEDS.LOG_PATH + "hasTags", hasTags);
                }
        
                public static void setCanShoot(boolean newCanShoot) {
                        canShoot = newCanShoot;
        }

                // rainbow for the X - mode (aka. Locking the wheels) // FLASH// 
                public void displayRainbow() {
                candle.setControl (rainbow);
                }

                public void setRainbow(boolean useRainbow) {
                this.useRainbow = useRainbow;
                }

                public void setSpindexerStuck(boolean spindexerStuck) {
                this.spindexerStuck = spindexerStuck;
        }

                public void setSpindexerStuck(){ // (1)
                        if (spindexerStuck) {
                                        candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT)
                                                .withColor(new RGBWColor(
                                                (int)(LEDS.HOT_PINK.red * 255),
                                                (int)(LEDS.HOT_PINK.green * 255),
                                                (int)(LEDS.HOT_PINK.blue * 255)
                                                )));
                        } else {
                        setOff();
                        }
                }

}
      
