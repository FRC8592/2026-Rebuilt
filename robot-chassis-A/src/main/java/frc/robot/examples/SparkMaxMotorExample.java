package frc.robot.examples;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

//this code will work for SparkFlex, SparkMax, and NEOs
public class SparkMaxMotorExample {
    //instantiate SparkMax motors here
    SparkMax motor;
    SparkMax leaderMotor;

    //configuration defines motor properties including inversion, soft limits, limit switches, and encoder properties
    SparkMaxConfig motorConfiguration;


    //the encoder on the motor will allow you to track the position of the subsystem by counting ticks on the motor
    RelativeEncoder encoder;

    public SparkMaxMotorExample(){
        motor = new SparkMax(9, MotorType.kBrushless);
        encoder = motor.getEncoder();

        //<------------ SET IDLE MODE ------------>
        motorConfiguration.idleMode(IdleMode.kBrake); //Use either kBrake or kCoast

        //<------------ INVERT MOTORS ------------>
        motorConfiguration.inverted(true); //<- this will invert your motor :)

        //<-------------- SET FOLLOWER MOTORS -------------->
        //first parameter is the motor you want to follow
        //second parameter is if you want to invert the motor or not 
        //i.e. if the motors move diff ways and you want to match the leader motor, it's true!
        motorConfiguration.follow(leaderMotor, true); 

        //<-------------- SET CURRENT LIMITS -------------->

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

        // motorConfiguration.closedLoop.positionWrappingInputRange(-Math.PI, Math.PI);



    }
    
}
