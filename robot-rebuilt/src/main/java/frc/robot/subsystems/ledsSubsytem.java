package frc.robot.subsystems;
//import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.configs.CANdleFeaturesConfigs;
import com.ctre.phoenix6.configs.LEDConfigs;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.LossOfSignalBehaviorValue;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import com.ctre.phoenix6.signals.VBatOutputModeValue;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.*;




public class ledsSubsytem {
    //TODO: RESTRICTED WHEELS, INTAKE SUCCESSFULLY LEDS, PROGRESS BAR FOR THE HOPPER, PERIOD OF THE GAME LED, CLIMB SUCESSFULLY
    //For the time periods during the game: AUTO (20 seconds), TRANSITION SHIFT (10 seconds), SHIFT 1 (25 seconds), SHIFT 2 (25 seconds), SHIFT 3 (25 seconds), SHIFT 4 (25 seconds), END GAME (30 seconds)
        private static CANdle candle;
        private static boolean intakeSucession;
        private static boolean hasFuel;
        private static boolean neturalMode;
        private static boolean isNeturalMode;
        private static double progressBarHopper = -1;
        private static int tagCount;
        private static boolean climbsucession;
        private static boolean isClimbing;
        private static boolean useRainbow;
        private static Timer timer = new Timer();
        private static Timer periodOfGame=new Timer();
        private static boolean restrictedWheels;
        private static RainbowAnimation rainbow = new RainbowAnimation(1,3);

    //COLORS going to be used: gray , red, blue, yellow, green, white, purple, rainbow

    public static void init(){
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.LED = new LEDConfigs()
            .withBrightnessScalar(1)
            .withStripType(StripTypeValue.GRB)
            .withLossOfSignalBehavior(LossOfSignalBehaviorValue.KeepRunning);
        configAll.CANdleFeatures = new CANdleFeaturesConfigs()
            .withStatusLedWhenActive(StatusLedWhenActiveValue.Enabled)
            .withVBatOutputMode(VBatOutputModeValue.Modulated);
        candle = new CANdle(0); // TODO: Change this value when the device name is giving.
        candle.getConfigurator().apply(configAll);
        timer.start();
    }

    //Creating the display for the LEDS
       public static void displayModeLEDs(){
        if(neturalMode){
            candle.setControl(
                new SolidColor(LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                    (int)(LEDS.WHITE.red*255),
                    (int)(LEDS.WHITE.green*255),
                    (int)(LEDS.WHITE.blue*255)
                ))
            );
        }
        else{
            candle. setControl(
                new SolidColor(LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                    (int)(LEDS.TEAL.red*255),
                    (int)(LEDS.TEAL.green*255),
                    (int)(LEDS.TEAL.blue*255)
                ))
            );
        }


    }

    // Will update to the periodOfGame
    public static void displayHasCoralLEDs(){ // need to change this statement
        if(!periodOfGame.hasElapsed(1) && periodOfGame.get()!=0){
            if((int)(timer.get()*10) % 2 == 0){ // need to change this if - else statement
                candle.setControl(
                new SolidColor (LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                (int)(LEDS.WHITE.red*255),
                (int)(LEDS.WHITE.green*255),
                (int)(LEDS.WHITE.blue*255)
                ))
            );
        }
            else{
                candle.setControl(
                new SolidColor (LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                (int)(LEDS.OFF.red*255),
                (int)(LEDS.OFF.green*255),
                (int)(LEDS.OFF.blue*255)
                ))
            );
            }
        }
        else{
                candle.setControl(
                new SolidColor (LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                (int)(LEDS.WHITE.red*255),
                (int)(LEDS.WHITE.green*255),
                (int)(LEDS.WHITE.blue*255)
                ))
            );
        }
    }

// Displaying the amount of the tags spotted, this might change cuase on the controls of the operator if they decided to reject auto - shoot
    public static void displayHasTagsLEDs(){
        if(tagCount>=2){
                candle.setControl(
                new SolidColor (LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                (int)(LEDS.GREEN.red*255),
                (int)(LEDS.GREEN.green*255),
                (int)(LEDS.GREEN.blue*255)
                ))
            );
        }
        else if(tagCount==1){
                candle.setControl(
                new SolidColor (LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                (int)(LEDS.YELLOW.red*255),
                (int)(LEDS.YELLOW.green*255),
                (int)(LEDS.YELLOW.blue*255)
                ))
            );
        }
        else if(tagCount==0){
                candle.setControl(
                new SolidColor (LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                (int)(LEDS.RED.red*255),
                (int)(LEDS.RED.green*255),
                (int)(LEDS.RED.blue*255)
                ))
            );
        }


        else if(tagCount ==-1){
            if((int)(timer.get()*3) % 2 == 0){
                candle.setControl(
                new SolidColor (LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                (int)(LEDS.RED.red*255),
                (int)(LEDS.RED.green*255),
                (int)(LEDS.RED.blue*255)
                ))
            );
            }


            else{
                candle.setControl(
                new SolidColor (LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                (int)(LEDS.OFF.red*255),
                (int)(LEDS.OFF.green*255),
                (int)(LEDS.OFF.blue*255)
                ))
            );
            }
        }


   
    }

    // While climbing lights will display and then RAINBOW
    public static void displayClimbingProgress(){
                candle.setControl(
                new SolidColor (LEDS.LED_CANDLE_COUNT, LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                (int)(LEDS.PURPLE.red*255),
                (int)(LEDS.PURPLE.green*255),
                (int)(LEDS.PURPLE.blue*255)
                ))
            );
    }

    // ProgressBar for the hopper based on fullness
    public static void displayProgressBarLEDs(){
                candle.setControl(
                new SolidColor (LEDS.LED_CANDLE_COUNT, (int) (LEDS.FULL_LED_COUNT*progressBarHopper)).withColor(new RGBWColor(
                (int)(LEDS.GREEN.red*255),
                (int)(LEDS.GREEN.green*255),
                (int)(LEDS.GREEN.blue*255)
                ))
            );
                    candle.setControl(
                new SolidColor ((int) (LEDS.FULL_LED_COUNT*progressBarHopper), LEDS.FULL_LED_COUNT).withColor(new RGBWColor(
                (int)(LEDS.RED.red*255),
                (int)(LEDS.RED.green*255),
                (int)(LEDS.RED.blue*255)
                ))
            );
    }

    // rainbow for the climb
    public static void displayRaindow(){
        candle.setControl(rainbow);
    }

    //intakesucessfully 
    public static void setHasFuel(boolean intakeSucession){
        hasFuel = intakeSucession;
    }

    //in Normal mode while the drivers are driving and there's no spefic mode 
    public static void setNeturalMode (boolean neturalMode){
        isNeturalMode = neturalMode;
    }

    //ProgressBar for the hopper 
    public static void progressBarHopper(double progress){
        progressBarHopper = progress;
    }

    // # of tags being seen by the camera 
    public static void setHasTags(int cameraTagCount){
        tagCount = cameraTagCount;
    }

    //climbing status
    public static void setclimbing(boolean climbsucession){
        isClimbing = climbsucession;
    }

    //the rainbow animation
    public static void setRainbow(boolean isRainbowAnimation){
        useRainbow = isRainbowAnimation; // set equal to isRainbowAnimation when ready to test
    }
   
   
}
   
