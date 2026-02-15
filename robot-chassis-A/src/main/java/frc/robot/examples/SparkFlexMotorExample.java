package frc.robot.examples;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

//this code will work for SparkFlex and NEO VORTEX
public class SparkFlexMotorExample {
    //instantiate SparkMax motors here
    SparkFlex motor;
    SparkFlex leaderMotor;

    //configuration defines motor properties including inversion, soft limits, limit switches, and encoder properties
    SparkFlexConfig motorConfiguration;

    //this controller will allow you to drive the motor using closed loop values (i.e. position and pid control)
    SparkClosedLoopController controller;

    //the encoder on the motor will allow you to track the position of the subsystem by counting ticks on the motor
    RelativeEncoder encoder;

    public SparkFlexMotorExample(){
        //In general, the motors we use will be Brushless. Brushed motors consume a lot of power and are complaint-worthy
        motor = new SparkFlex(9, MotorType.kBrushless);
        leaderMotor = new SparkFlex(10, MotorType.kBrushless);

        //instantiate the motor configuration
        motorConfiguration = new SparkFlexConfig();

        //instantiate the motor contoroller
        controller = motor.getClosedLoopController();

        //get the encoder from the motor 
        encoder = motor.getEncoder();

        //<------------ SET IDLE MODE ------------>
        motorConfiguration.idleMode(IdleMode.kBrake); //Use either kBrake or kCoast

        //<------------ INVERT MOTORS ------------>
        motorConfiguration.inverted(true); //<- this will invert your motor :)

        //<-------------- SET FOLLOWER MOTORS -------------->
        //first parameter is the motor you want to follow
        //second parameter is if you want to invert the motor or not 
        //i.e. if the motors move diff ways and you want to match the leader motor, put true
        motorConfiguration.follow(leaderMotor, true); 

        //<-------------- SET CURRENT LIMITS -------------->
        //the default is 80a. Choose the appropriate method below for your needs.
        motorConfiguration.smartCurrentLimit(0);
        motorConfiguration.smartCurrentLimit(0, 0);
        motorConfiguration.smartCurrentLimit(0, 0, 0);

        //<-------------- SET SOFT LIMITS -------------->
        motorConfiguration.softLimit.forwardSoftLimitEnabled(true); 
        motorConfiguration.softLimit.forwardSoftLimit(10); 
        motorConfiguration.softLimit.reverseSoftLimitEnabled(true); 
        motorConfiguration.softLimit.reverseSoftLimit(0);

        //<-------------- SET PID VALUES -------------->
        //You typically will only need to use one slot for PID
        //You can utilize different slots(ex. slot1, slot2) to create different pids for the same motor
        //i.e. different pid values for an arm holding heavy objects vs. with no objects  
        motorConfiguration.closedLoop.pid(0, 0, 0, ClosedLoopSlot.kSlot0);

        //APPLY CONFIGURATION TO MOTOR - MUST DO WHENEVER YOU CHANGE CONFIGURATION SETTINGS
        //Use ResetMode.kResetSafeParameters and persistMode.kPersistParameters to ensure configuration consistency
        motor.configure(motorConfiguration, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

    //basic method to run the motor without any type of control
    public void run(){
        motor.set(1); //set speed of motor from -1 to 1
    }

    //method to run the motor using velocity control
    public void runVelocityControl(){
        //the setControl method utilizes closed loop (meaning that it will use the pid values)
        //other methods such as setVoltage() and set() will NOT use the closed loop settings

        //the setpoint value is in RPM
        //ControlType is velocity, which is best for when you want to run a motor up to a set velocity and position doesn't matter
        //Slot is the slot you chose for your pid values
        controller.setSetpoint(90, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    //method to run the motor using position control
    public void runPositionControl(){
        //the setpoint value is in rotations
        //ControlType is position, which is best for position-based mechanisms (arm, turret, etc)
        controller.setSetpoint(90, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    //basic method to stop the motor
    public void stop(){
        motor.stopMotor();
    }

    //will hold the motor at current position
    public void stopWithControl(){
        controller.setSetpoint(getPosition(), ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    //returns current velocity in rotations per minute
    public double getVelocityRPM(){
        return encoder.getVelocity();
    }

    //returns current position in motor rotations
    public double getPosition(){
        return encoder.getPosition();
    }

    //sets the current rotation of the motor as '0'
    public void zeroPosition(){
        encoder.setPosition(0);
    }

    //returns if the motor reached its forward soft limit
    public boolean atForwardLimit(){
        return motor.getForwardSoftLimit().isReached();
    }

    //returns if the motor reached its reverse soft limit
    public boolean atReverseLimit(){
        return motor.getReverseSoftLimit().isReached();
    }
    
}
