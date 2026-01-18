package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.hardware.CANdle.*;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.ctre.phoenix6.hardware.*;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;

public class LEDS {
    private static CANdle candle;
    private static boolean hasCoral; 
    private static boolean coralMode;
    private static double progressBar = -1;
    private static int tagCount;
    private static boolean deepclimb;
    private static boolean useRainbow;
    private static Timer timer = new Timer(); 
    private static Timer ledTimer=new Timer();
    //private static RainbowAnimation rainbow = new RainbowAnimation(1,3,LEDS.FULL_LED_COUNT);
    // private static Trigger coralScore = new Trigger(() -> hasCoral);

    // COLORS going to be used: gray , red, blue, yellow, green, white, purple, rainbow

    public static void init(){
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.LED = new LEDConfigs()
            .withBrightnessScalar(1)
            .withStripType(StripTypeValue.GRB)
            .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning);
        configAll.CANdleFeatures = new CANdleFeaturesConfigs()
            .withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled)
            .withVBatOutputMode(VBatOutputModeValue.Modulated);
        candle = new CANdle(46);
        candle.getConfigurator().apply(configAll);
        timer.start();

    }

    public static void writeLEDs(){
       
    }

    public static void displayModeLEDs(){

    }

    public static void displayHasCoralLEDs(){
    }

    public static void displayHasTagsLEDs(){

    
    }
    public static void displayDeepClimb(){
    }

    public static void displayProgressBarLEDs(){
        
    }

    public static void displayRaindow(){
    }

    public static void setHasCoral(boolean robotHasCoral){
    }

    public static void setCoralMode(boolean isCoralMode){
    }

    public static void setProgressBar(double progress){
    }

    public static void setHasTags(int cameraTagCount){
    }

    public static void setDeepClimb(boolean isDeepClimb){
    }

    public static void setRainbow(boolean isRainbowAnimation){
    }
    
    
}
    