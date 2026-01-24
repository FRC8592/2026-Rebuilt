package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import java.lang.Math;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;
import frc.robot.helpers.motor.spark.SparkFlexMotor;


public class Intake extends SubsystemBase{
    private SparkFlexMotor IntakeMotor; 
    private PIDProfile MotorPID;
 

    public Intake() {
        // Create motor and assign PID values from the CONSTANTS class
        IntakeMotor = new SparkFlexMotor(INTAKE.INTAKE_MOTOR_CAN_ID, false);
        MotorPID = new PIDProfile();
        MotorPID.setSlot(0);
        MotorPID.setPID(INTAKE.INTAKE_P, INTAKE.INTAKE_I, INTAKE.INTAKE_D);
        IntakeMotor.withGains(MotorPID);

        //TODO: Determine an appropriate current limit for the intake motor
        IntakeMotor.setCurrentLimit(80);

        //TODO: For tuning, put the PID and velocity values on the dashboard.  Remove before competition
        SmartDashboard.putNumber("P_INTAKE", INTAKE.INTAKE_P);
        SmartDashboard.putNumber("I_INTAKE", INTAKE.INTAKE_I);
        SmartDashboard.putNumber("D_INTAKE", INTAKE.INTAKE_D);
        SmartDashboard.putNumber("Vi_INTAKE",INTAKE.INTAKE_VI);
    }

    // Main method for running the intake under velocity control
    public void runAtSpeed() {
        double RPM = SmartDashboard.getNumber("Vi_INTAKE", INTAKE.INTAKE_VI); // TODO: Remove this before competition
        IntakeMotor.setPercentOutput(0.8);
    }

    // Main command for running the intake under velocity control
    public Command runAtSpeedCommand() {
        return this.runOnce(() -> runAtSpeed());
    }

    //
    // Update the PID constants for the Intake motor
    //
    // The Neo Vortex motors will not accept a change to the PID parameters while running.
    // Thusly, this method must be called from disabledPeriod() in Robot.java.
    //
    public void updatePID() {
        //System.out.println("Going into updatePID Method");
        double P = SmartDashboard.getNumber("P_INTAKE", 0.001);
        double I = SmartDashboard.getNumber("I_INTAKE", 0.0);
        double D = SmartDashboard.getNumber("D_INTAKE", 0.0);
    
        MotorPID.setPID(P, I, D);
        IntakeMotor.withGains(MotorPID);
        }
    

    //
    // Stop motor using percent output.  We do this so that it will slow to a stop naturally instead of using battery power to stop quickly
    //
    public void stop() {
        IntakeMotor.setPercentOutput(0);
    }

    // Stop motor
    public Command stopCommand(){
        return this.runOnce(() -> stop());
    }

    // Get the current velocity of the intake motor in RPM
    public double getVelocity(){
        return IntakeMotor.getVelocityRPM();
    }
    //This is simply for calculation to get the ball landing in the center of the goal, based on the distance to the hub
    //This is very much a theoretical implementation, simply putting in just the math
    // public double DistanceToRPM(double theta, double dis){
    //     return (1/Math.cos(theta))* (Math.sqrt((0.5 * 9.81 * Math.pow(dis,2))/(Intake.Intake_HEIGHT + Math.tan(theta) - Intake.HUB_HEIGHT)));
    // }

    @Override
    public void periodic(){
        Logger.recordOutput("Intake Motor Velocity RPM", getVelocity());
    }
        
}