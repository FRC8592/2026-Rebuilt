package frc.robot.examples;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

//This code will work for Falcon 500s, Krakenx44, and Krakenx60 motors.
public class TalonFXMotorExample {
    private TalonFX motor = new TalonFX(9); 
    //didn't create configuration for followerMotor, but you should create configurations for every motor! 
    private TalonFX followerMotor = new TalonFX(10); 

    //configuration defines motor properties including inversion, pid, and current limits
    private TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

    //Below are different control types that you can utilize (so don't use multiple at once)

    //this is the controller that will drive the motor using velocity controls
    //you should use this when you care about the motor speed but not which position it ends at
    //i.e. flywheels, conveyer belts 
    private VelocityVoltage velocityRequest = new VelocityVoltage(0);

    //this is the controller that can drive the motor using motion magic, and also uses FOC (field-oriented control)
    //this is in essance an improved version of VelocityVoltage
    //i.e. conveyer belts with gentle start and stop to prevent mechanical duress
    private MotionMagicVelocityVoltage motionMagicRequest = new MotionMagicVelocityVoltage(0);

    //this is the controller that drives the motor to a certain position
    //i.e. arm with set positions, elevators with set heights, intakes that retract
    private PositionVoltage positionRequest = new PositionVoltage(0);

    /**
     * In the constructor of your susbsystem class, you will want to set all your motor configurations and apply 
     */
    public TalonFXMotorExample(){
        //<------------ SET IDLE MODE ------------>
        //Brake: doesn't move in disabled, "brakes" to a stop
        //Coast: moves in disabled, "cruises" to a stop
        motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        //<------------ INVERT MOTORS ------------>
        //if the motor is inverted, set to Clockwise_Positive
        //the default value is InvertedValue.CounterClockwise_Positive
        motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // <- i.e. this inverts the motor

        //<-------------- SET FOLLOWER MOTORS -------------->
        //use MotorAlignmentValue.Aligned when the motors spin in the same direction mechanically
        //use MotorAlignmentValue.Opposed when the motors spin in the opposite direction mechanically
        followerMotor.setControl(new Follower(motor.getDeviceID(), MotorAlignmentValue.Aligned));

        //<-------------- SET CURRENT LIMITS -------------->
        motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true; 
        motorConfiguration.CurrentLimits.StatorCurrentLimit = 20.0; //set the current limit value

        //<-------------- SET SOFT LIMITS -------------->
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitEnable = true; 
        motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold = 10.0; //threshold in rotations (the "highest" position)

        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0.0; //threshold in rotations (the "lowest" position)

        //<-------------- SET PID VALUES -------------->
        //You typically will only need to use one slot for PID
        //You can utilize different slots(ex. slot1, slot2) to create different pids for the same motor
        //i.e. different pid values for an arm holding heavy objects vs. with no objects  
        motorConfiguration.Slot0.kP = 0.0; 
        motorConfiguration.Slot0.kI = 0.0;
        motorConfiguration.Slot0.kD = 0.0;

        //<-------------- CONFIGURE MOTION MAGIC -------------->
        //Motion Magic provides smoother acceleration/deacceleration, and can build on top of pid or be used independently
        //this will provide trapezoidal profile (if jerk is zero) or an s-curve profile (if jerk is non-zero)
        double targetCruiseVelocity = 80; //Rotations per Second
        double targetAcceleration = 160; //Rotations per Second / Second (160rps/s is 0.5 seconds)
        double targetJerk = 1600; //Rotations per Second / Second / Second (1600 rps/s/s is 0.1 seconds)
        motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = targetCruiseVelocity;
        motorConfiguration.MotionMagic.MotionMagicAcceleration = targetAcceleration;
        motorConfiguration.MotionMagic.MotionMagicJerk = targetJerk;

        //APPLY CONFIGURATION TO MOTOR - MUST DO WHENEVER YOU CHANGE CONFIGURATION SETTINGS
        motor.getConfigurator().apply(motorConfiguration);

        //if you're using position control, you might want to zero its position when you start the robot
        motor.setPosition(0);

    }

    public void runVelocityControl(double rps){
        //USING THE VELOCITY VOLTAGE CONTROLLER
        //the setControl method utilizes closed loop (meaning that it will use the pid values)
        //other methods such as setVoltage() and set() will NOT use the closed loop settings

        //withSlot() defines the pid slot
        //withVelocity() is the new velocity in RPS(Rotations per Second)
        //withEnableFOC() should generally be false
        motor.setControl(velocityRequest.withSlot(0)
                                        .withVelocity(rps)
                                        .withEnableFOC(false));
    }

    public void runMotionMagicVelocityControl(double rps){
        //withEnableFOC() should be set as true becaue the controller uses it to be more effective
        motor.setControl(motionMagicRequest.withSlot(0)
                                        .withVelocity(rps)
                                        .withEnableFOC(true));
    }

    public void runPositionControl(double position){
        motor.setControl(positionRequest.withPosition(position)
                                        .withSlot(0));
    }

    //stops motors (both position and velocity control)
    public void stop() {
        motor.setControl(velocityRequest.withSlot(0).withVelocity(0));
    }

    //zero the motor's encoder (useful for arms/elevators at known limit
    public void zeroPosition() {
        motor.setPosition(0);
    }

    //returns current velocity in rotations per second 
    public double getVelocityRPS() {
        return motor.getVelocity().getValueAsDouble();
    }

    //returns current motor position in rotations
    public double getPositionRotations() {
        return motor.getPosition().getValueAsDouble();
    }

    //true if motor is at forward soft limit
    public boolean atForwardLimit() {
        return getPositionRotations() >= motorConfiguration.SoftwareLimitSwitch.ForwardSoftLimitThreshold;
    }

    //true if motor is at reverse soft limit
    public boolean atReverseLimit() {
        return getPositionRotations() <= motorConfiguration.SoftwareLimitSwitch.ReverseSoftLimitThreshold;
    }

}