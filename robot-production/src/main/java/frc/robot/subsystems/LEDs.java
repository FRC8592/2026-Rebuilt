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
import frc.robot.CodeConstants;
import frc.robot.Constants;
import frc.robot.Constants.LEDS;

public class LEDs extends SubsystemBase {
        // TODO: RESTRICTED WHEELS, INTAKE SUCCESSFULLY LEDS, PROGRESS BAR FOR THE
        private CANdle candle;
        private static boolean canShoot = false; 
        private int hasTags;

                public LEDs() {
                        CANdleConfiguration configAll = new CANdleConfiguration();
                        configAll.LED = new LEDConfigs().withBrightnessScalar(1)
                                        .withStripType(StripTypeValue.GRB)
                                        .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning);
                        configAll.CANdleFeatures = new CANdleFeaturesConfigs()
                                        .withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled)
                                        .withVBatOutputMode(VBatOutputModeValue.Modulated);
                        candle = new CANdle(LEDS.LEDS_CAN_ID);
                        candle.getConfigurator().apply(configAll);
                }
        
                // SETTERS 
                public void setCanShoot() {
                        candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, CodeConstants.LEDS.FULL_LED_COUNT)
                                        .withColor(new RGBWColor((int) (Constants.LEDS.TEAL.red * 255),
                                                        (int) (Constants.LEDS.TEAL.green * 255),
                                                        (int) (Constants.LEDS.TEAL.blue * 255))));
                }
        
                public void setCannotShoot() {
                        candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, CodeConstants.LEDS.FULL_LED_COUNT)
                                        .withColor(new RGBWColor((int) (Constants.LEDS.ORANGE.red * 255),
                                                        (int) (Constants.LEDS.ORANGE.green * 255),
                                                        (int) (Constants.LEDS.ORANGE.blue * 255))));
                }
        
                public void setOff() {
                        candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, CodeConstants.LEDS.FULL_LED_COUNT)
                                        .withColor(new RGBWColor((int) (Constants.LEDS.OFF.red * 255),
                                                        (int) (Constants.LEDS.OFF.green * 255),
                                                        (int) (Constants.LEDS.OFF.blue * 255))));
                }
        
                public void displayindexerRunningForward() { 
                        candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, CodeConstants.LEDS.FULL_LED_COUNT)
                                        .withColor(new RGBWColor((int) (Constants.LEDS.WHITE.red * 255),
                                                        (int) (Constants.LEDS.WHITE.green * 255),
                                                        (int) (Constants.LEDS.WHITE.blue * 255))));
                }

                public void displayindexerRunningReverse() {
                        candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, CodeConstants.LEDS.FULL_LED_COUNT)
                                        .withColor(new RGBWColor((int) (Constants.LEDS.PURPLE.red * 255),
                                                        (int) (Constants.LEDS.PURPLE.green * 255),
                                                        (int) (Constants.LEDS.PURPLE.blue * 255))));
                }
        
                public void displayHasTagsLEDs() {
                        if (hasTags >= 2) {
                                candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, CodeConstants.LEDS.FULL_LED_COUNT)
                                                .withColor(new RGBWColor((int) (Constants.LEDS.GREEN.red * 255),
                                                                (int) (Constants.LEDS.GREEN.green * 255),
                                                                (int) (Constants.LEDS.GREEN.blue * 255))));
                        } else if (hasTags == 1) {
                                candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, CodeConstants.LEDS.FULL_LED_COUNT)
                                                .withColor(new RGBWColor((int) (Constants.LEDS.YELLOW.red * 255),
                                                                (int) (Constants.LEDS.YELLOW.green * 255),
                                                                (int) (Constants.LEDS.YELLOW.blue * 255))));
                        } else {
                                candle.setControl(new SolidColor(LEDS.LED_CANDLE_COUNT, CodeConstants.LEDS.FULL_LED_COUNT)
                                                .withColor(new RGBWColor((int) (Constants.LEDS.RED.red * 255),
                                                                (int) (Constants.LEDS.RED.green * 255),
                                                                (int) (Constants.LEDS.RED.blue * 255))));
                        }
                }
        
                public void setHasTags(int newHasTags) {
                        hasTags = newHasTags;
                        Logger.recordOutput(Constants.LEDS.LOG_PATH + "hasTags", hasTags);
                }
        
                public void setCanShootState(boolean newCanShoot) {
                this.canShoot = newCanShoot;
                }

                
}
      