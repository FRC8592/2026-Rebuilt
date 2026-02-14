package frc.robot.examples;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

//this code will work for SparkFlex, SparkMax, and NEOs
public class REVMotorExample {
    SparkMax motor;
    SparkMaxConfig motorConfiguration;
    RelativeEncoder encoder;

    public REVMotorExample(){
        motor = new SparkMax(9, MotorType.kBrushless);
        encoder = motor.getEncoder();

    }
    
}
