package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SteadyStateKalmanFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.lang.management.MemoryNotificationInfo;

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
        // spinnerMotor = new SparkFlexMotor(INDEXER.SPINNER_CAN_ID, false);
        // outputMotor = new SparkFlexMotor(INDEXER.OUTPUT_CAN_ID, false);

        // MotorPID = new PIDProfile();
        // MotorPID.setSlot(0);
        // MotorPID.setPID(INDEXER.SPINNER_P, INDEXER.SPINNER_I,INDEXER.SPINNER_D);
        // spinnerMotor.withGains(MotorPID);
        
        // MotorPID.setSlot(0);
        // MotorPID.setPID(INDEXER.OUTPUT_P, INDEXER.OUTPUT_I,INDEXER.OUTPUT_D);
        // outputMotor.withGains(MotorPID);
  
        // TODO: set idle modes
        //spinnerMotor.setIdleMode(IdleMode.kCoast);
        //outputMotor.setIdleMode(IdleMode.kCoast);

        // TODO: Determine an appropriate current limit for the indexer motors
        spinnerMotor.setCurrentLimit(INDEXER.SPINNER_CURRENT_LIMIT);
        outputMotor.setCurrentLimit(INDEXER.OUTPUT_CURRENT_LIMIT);

        // TODO: For tuning, put the PID and velocity values on the dashboard.  Remove before competition
        SmartDashboard.putNumber("P_SPINNER", INDEXER.SPINNER_P);
        SmartDashboard.putNumber("I_SPINNER", INDEXER.SPINNER_I);
        SmartDashboard.putNumber("D_SPINNER", INDEXER.SPINNER_D);
        SmartDashboard.putNumber("Vi_SPINNER", INDEXER.SPINNER_VI);

        SmartDashboard.putNumber("P_OUTPUT", INDEXER.OUTPUT_P);
        SmartDashboard.putNumber("I_OUTPUT", INDEXER.OUTPUT_I);
        SmartDashboard.putNumber("D_OUTPUT", INDEXER.OUTPUT_D);
        SmartDashboard.putNumber("Vi_OUTPUT", INDEXER.OUTPUT_VI);
    }


    /**
     * Run the indexer at a set speed
     */
    public void runAtSpeed(){
        double RPM_SPINNER = SmartDashboard.getNumber("Vi_SPINNER", INDEXER.SPINNER_VI);
        double RPM_OUTPUT = SmartDashboard.getNumber("Vi_OUTPUT", INDEXER.OUTPUT_VI);

        spinnerMotor.setVelocity(RPM_SPINNER);
        // TODO: Put output motor into velocity control
        //outputMotor.setVelocity(RPM_OUTPUT);
        outputMotor.setPercentOutput(1.0);
    }


    /**
     * Command to run the indexer at a set speed
     * @return
     */
    public Command runAtSpeedCommand(){
        return this.runOnce(() -> runAtSpeed());
    }


    /**
     * Update the PID constants for the indexer motors from SmartDashboard values
     * 
     * The Neo Vortex motors will not accept a change to the PID parameters while running.
     * Thusly, this method must be called from disabledPeriod() in Robot.java.
     */
    public void updatePID(){
        //System.out.println("Going into updatePID Method");
        double P_SPINNER = SmartDashboard.getNumber("P_SPINNER", INDEXER.SPINNER_P);
        double I_SPINNER = SmartDashboard.getNumber("I_SPINNER", INDEXER.SPINNER_I);
        double D_SPINNER = SmartDashboard.getNumber("D_SPINNER", INDEXER.SPINNER_D);

        double P_OUTPUT = SmartDashboard.getNumber("P_OUTPUT", INDEXER.OUTPUT_P);
        double I_OUTPUT = SmartDashboard.getNumber("I_OUTPUT", INDEXER.OUTPUT_I);
        double D_OUTPUT = SmartDashboard.getNumber("D_OUTPUT", INDEXER.OUTPUT_D);

        MotorPID.setSlot(0);
        MotorPID.setPID(P_SPINNER, I_SPINNER, D_SPINNER);
        spinnerMotor.withGains(MotorPID);

        MotorPID.setSlot(0);
        MotorPID.setPID(P_OUTPUT, I_OUTPUT, D_OUTPUT);
        outputMotor.withGains(MotorPID);
    }


    /**
     * Stop the indexer motors
     * 
     * The spinnner can spin down naturally
     * The output motor must stop immeidately so we do not continue to feed the shooter
     */
    public void stop(){
        spinnerMotor.setPercentOutput(0);
        outputMotor.setVelocity(0.0);
    }


    /**
     * Stop command for the indexer motors
     * @return stop command
     */
    public Command stopCommand(){
        return this.runOnce(() -> stop());
    }


    /**
     * Get the velocity of the spinner motor in RPM
     * @return velocity in RPM
     */
    public double getVelocitySpinner(){
        return spinnerMotor.getVelocityRPM();
    }

    /**
     * Get the velocity of the output motor in RPM
     * @return velocity in RPM
     */
    public double getVelocityOutput(){
        return outputMotor.getVelocityRPM();
    }

    /**
     * Periodic method, primarily used for logging
     */
    @Override
    public void periodic(){
        Logger.recordOutput("Spinner RPM", getVelocitySpinner());
        Logger.recordOutput("Output RPM", getVelocityOutput());
    }
        
}