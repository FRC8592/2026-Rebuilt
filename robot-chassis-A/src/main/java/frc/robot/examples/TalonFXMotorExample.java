package frc.robot.examples;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class TalonFXMotorExample {
    //instantiate a motor object
    //Also called "Falcon" motors, they have a max RPM of 6380 and the encoder returns feedback 2048 steps per rotation
    private TalonFX motor = new TalonFX(9); //CAN-ID goes into constructor
    private TalonFX followerMotor = new TalonFX(10);

    //this configuration will define motor properties including inversion, pid, and current limits.
    private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    //this is the controller that can be used to drive the motor using the pid values in the configuration
    //VelocityVoltage takes the velocity of the motor - set as 0 intially (won't move)
    private VelocityVoltage velocityRequest = new VelocityVoltage(0);

    //this is the controller that can drive the motor using motion magic, and also integrates FOC (field-oriented control)
    private MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);

    /**
     * In the constructor of your susbsystem class, you will want to set all your motor configurations and apply 
     */
    public TalonFXMotorExample(){
        //SET IDLE MODE
        //Brake: doesn't move in disabled, "brakes" to a stop
        //Coast: moves in disabled, "cruises" to a stop 
        motor.setNeutralMode(NeutralModeValue.Brake); //can use NeutralModeValue.Coast for CoastMode
        //can also set idle mode with the following
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //INVERTING MOTORS
        //if the motor is inverted (rotates the opposite way!), set to Clockwise_Positive
        //the default value is InvertedValue.CounterClockwise_Positive
        motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // <- i.e. this inverts the motor

        //SET FOLLOWER MOTORS
        //use MotorAlignmentValue.Aligned when the motors spin in the same direction mechanically
        //use MotorAlignmentValue.Opposed when the motors spin in opposite directions mechanically
        followerMotor.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Aligned));

        //SET CURRENT LIMITS
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true; //default value is true, but enable once
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 20.0; //set the current limit value

        //SETTING SOFT LIMITS
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; //enable soft limits for forward motion 
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 0.0; //position threshold in rotations (the "highest" position)

        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true; //enable soft limits for backward motion 
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; //position threshold in rotations (the "lowest" position)

        //SETTING PID VALUES
        //You typically will only need to use one slot for PID
        //You can utilize different slots to create multiple pid settings for the same motor if needed
        //i.e. different pid values for an arm holding heavy objects vs. with no objects  
        motorConfiguration.Slot0.kP = 0.0; 
        motorConfiguration.Slot0.kI = 0.0;
        motorConfiguration.Slot0.kD = 0.0;

        //CONFIGURE MOTION MAGIC
        //Motion Magic provides smoother acceleration/deacceleration, and can build on top of pid or be used independently
        //this can provide trapezoidal profile (if jerk is zero) or an s-curve velocity profile (if jerk is non-zero)
        double targetCruiseVelocity = 80; //Unit is Rotations per Second
        double targetAcceleration = 160; //Unit is Rotations per Second / Second (160rps/s is 0.5 seconds)
        double targetJerk = 1600; //Unit is Rotations per Second / Second / Second (1600 rps/s/s is 0.1 seconds)
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = targetCruiseVelocity;
        motorConfiguration.MotionMagic.MotionMagicAcceleration = targetAcceleration;
        motorConfiguration.MotionMagic.MotionMagicJerk = targetJerk;

        //APPLY CONFIGURATION TO MOTOR - MUST DO WHENEVER YOU CHANGE CONFIGURATION SETTINGS
        motor.getConfigurator().apply(motorConfiguration);

    }

    /**
     *  A method to run the motor
     * */
    public void run(){
        //USING THE VELOCITY VOLTAGE CONTROLLER
        //the setControl method utilizes the closed loop settings (meaning that it will use the pid values)
        //other methods such as setVoltage() and set() will NOT use the closed loop settings

        //withSlot() defines the pid slot
        //withVelocity() is the new velocity in RPS(Rotations per Second)
        //withEnableFOC() can enable FOC, or Field-Oriented Control, which increases motor performance by ~15%. 
        motor.setControl(velocityRequest.withSlot(0)
                                        .withVelocity(80)
                                        .withEnableFOC(false));

        //USING THE MOTION MAGIC CONTROLLER
        //withEnableFOC() should be set as true, because the controller will use it effectively
        motor.setControl(motionMagicRequest.withSlot(0)
                                            .withVelocity(80)
                                            .withEnableFOC(true));
    }

    /**
     * A method to stop the motor
     */
    public void stop(){
        //USING THE VELOCITY VOLTAGE CONTROLLER
        motor.setControl(velocityRequest.withSlot(0)
                                        .withVelocity(0));

        //USING THE MOTION MAGIC CONTROLLER
        motor.setControl(motionMagicRequest.withSlot(0)
                                           .withVelocity(0));

    }


}
