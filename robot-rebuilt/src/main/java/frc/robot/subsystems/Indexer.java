package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.spark.SparkFlexMotor;


public class Indexer extends SubsystemBase{
    private SparkFlexMotor spinnerMotor;
    private SparkFlexMotor outputMotor;
    private PIDProfile MotorPID;

    /**
     * Constructor for the Indexer subsystem
     * 
     * Instatiate the motors with initial PID values from the CONSTANTS class
     */
    public Indexer(){
        spinnerMotor = new SparkFlexMotor(INDEXER.INDEXER_SPINNER_CAN_ID, false);
        outputMotor = new SparkFlexMotor(INDEXER.INDEXER_OUTPUT_CAN_ID, false);

        MotorPID = new PIDProfile();
        MotorPID.setSlot(0);
        MotorPID.setPID(INDEXER.INDEXER_SPINNER_P, INDEXER.INDEXER_SPINNER_I,INDEXER.INDEXER_SPINNER_D);
        spinnerMotor.withGains(MotorPID);
        
        MotorPID.setPID(INDEXER.INDEXER_OUTPUT_P, INDEXER.INDEXER_OUTPUT_I,INDEXER.INDEXER_OUTPUT_D);
        outputMotor.withGains(MotorPID);
  
        // set idle modes
        // spinnerMotor.setIdleMode(IdleMode.kCoast);
        // outputMotor.setIdleMode(IdleMode.kCoast);

        // TODO: Determine an appropriate current limit for the indexer motors
        spinnerMotor.setCurrentLimit(80);
        outputMotor.setCurrentLimit(80);

        // TODO: For tuning, put the PID and velocity values on the dashboard.  Remove before competition
        SmartDashboard.putNumber("P_SPINNER", INDEXER.INDEXER_SPINNER_P);
        SmartDashboard.putNumber("I_SPINNER", INDEXER.INDEXER_SPINNER_I);
        SmartDashboard.putNumber("D_SPINNER", INDEXER.INDEXER_SPINNER_D);
        SmartDashboard.putNumber("Vi_SPINNER", INDEXER.INDEXER_SPINNER_VELOCITY);

        SmartDashboard.putNumber("P_OUTPUT", INDEXER.INDEXER_OUTPUT_P);
        SmartDashboard.putNumber("I_OUTPUT", INDEXER.INDEXER_OUTPUT_I);
        SmartDashboard.putNumber("D_OUTPUT", INDEXER.INDEXER_OUTPUT_D);
        SmartDashboard.putNumber("Vi_OUTPUT", INDEXER.INDEXER_OUTPUT_VELOCITY);
        //LeftIndexerMotor.configureMotionMagic(Indexer.MAX_ACCELERATION, Indexer.CRUISE_VELOCITY);
    }

    public void runAtSpeed(){
        double RPM_SPINNER = SmartDashboard.getNumber("Vi_SPINNER", 0);
        double RPM_OUTPUT = SmartDashboard.getNumber("Vi_OUTPUT", 0);
        spinnerMotor.setPercentOutput(1);
        outputMotor.setVelocity(RPM_OUTPUT);
    }

    public Command runAtSpeedCommand(){
        return this.runOnce(() -> runAtSpeed());
    }
    public void updatePID(){
        //System.out.println("Going into updatePID Method");
        double P_SPINNER = SmartDashboard.getNumber("P_SPINNER", 0.001);
        double I_SPINNER = SmartDashboard.getNumber("I_SPINNER", 0.0);
        double D_SPINNER = SmartDashboard.getNumber("D_SPINNER", 0.0);

        double P_OUTPUT = SmartDashboard.getNumber("P_OUTPUT", 0.001);
        double I_OUTPUT = SmartDashboard.getNumber("I_OUTPUT", 0.0);
        double D_OUTPUT = SmartDashboard.getNumber("D_OUTPUT", 0.0);
        MotorPID.setPID(P_SPINNER, I_SPINNER, D_SPINNER);
        spinnerMotor.withGains(MotorPID);

        MotorPID.setPID(P_OUTPUT, I_OUTPUT, D_OUTPUT);
        outputMotor.withGains(MotorPID);
    }
    public void stop(){
        spinnerMotor.setPercentOutput(0);
        outputMotor.setPercentOutput(0);
    }

    public Command stopCommand(){
        return this.runOnce(() -> stop());
    }

    public double getVelocitySpinner(){
        return spinnerMotor.getVelocityRPM();
    }

    public double getVelocityOutput(){
        return outputMotor.getVelocityRPM();
    }
    //This is simply for calculation to get the ball landing in the center of the goal, based on the distance to the hub
    //This is very much a theoretical implementation, simply putting in just the math
    // public double DistanceToRPM(double theta, double dis){
    //     return (1/Math.cos(theta))* (Math.sqrt((0.5 * 9.81 * Math.pow(dis,2))/(Indexer.Indexer_HEIGHT + Math.tan(theta) - Indexer.HUB_HEIGHT)));
    // }

    @Override
    public void periodic(){
        // updatePID();
        Logger.recordOutput("Motor Velocity(1) RPM", getVelocitySpinner());
        Logger.recordOutput("Motor Velocity(2) RPM", getVelocityOutput());
    }
        
}