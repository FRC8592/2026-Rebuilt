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


public class Intake extends SubsystemBase{
    private SparkFlexMotor IntakeMotor; 
    private PIDProfile MotorPID;
 
    /**
     * Constructor for the Intake subsystem
     * 
     * Instatiate the motor with initial PID values from the CONSTANTS class
     */
    public Intake() {
        // IntakeMotor = new SparkFlexMotor(INTAKE.INTAKE_MOTOR_CAN_ID, false);
        // MotorPID = new PIDProfile();
        // MotorPID.setSlot(0);
        // MotorPID.setPID(INTAKE.INTAKE_P, INTAKE.INTAKE_I, INTAKE.INTAKE_D);
        // IntakeMotor.withGains(MotorPID);

        // TODO: Set idle mode
        //IntakeMotor.setIdleMode(IdleMode.kCoast);
  
        // TODO: Determine an appropriate current limit for the intake motor
        // IntakeMotor.setCurrentLimit(INTAKE.INTAKE_CURRENT_LIMIT);

        // TODO: For tuning, put the PID and velocity values on the dashboard.  Remove before competition
        SmartDashboard.putNumber("P_INTAKE", INTAKE.INTAKE_P);
        SmartDashboard.putNumber("I_INTAKE", INTAKE.INTAKE_I);
        SmartDashboard.putNumber("D_INTAKE", INTAKE.INTAKE_D);
        SmartDashboard.putNumber("Vi_INTAKE",INTAKE.INTAKE_VI);
    }


//     /**
//      * Run the intake at a set speed
//      */
//     public void runAtSpeed() {
//         double RPM = SmartDashboard.getNumber("Vi_INTAKE", INTAKE.INTAKE_VI); // TODO: Remove this before competition
//         // IntakeMotor.setPercentOutput(0.8); // TODO: Temporary fix to prevent oscillation.  Tune PID and run in velocity mode.
//         // TODO: Tune PID for the intake to prevent oscillation.  
//         // IntakeMotor.setVelocity(RPM);
//     }


//     /**
//      * Command to run the intake at a set speed
//      */
//     // public Command runAtSpeedCommand() {
//     //     return this.runOnce(() -> runAtSpeed());
//     // }


//     /**
//      * Update the PID constants for the intake motor from SmartDashboard values
//      * 
//      * The Neo Vortex motors will not accept a change to the PID parameters while running.
//      * Thusly, this method must be called from disabledPeriod() in Robot.java.
//      */
//     public void updatePID() {
//         double P = SmartDashboard.getNumber("P_INTAKE", INTAKE.INTAKE_P);
//         double I = SmartDashboard.getNumber("I_INTAKE", INTAKE.INTAKE_I);
//         double D = SmartDashboard.getNumber("D_INTAKE", INTAKE.INTAKE_D);
    
//         MotorPID.setSlot(0);
//         MotorPID.setPID(P, I, D);
//         IntakeMotor.withGains(MotorPID);
//         }
    

//     /**
//      * Stop the intake motor
//      * 
//      * We do this using % ouptput so that the motor will slow to a stop naturally
//      * Using setVelocity() will cause the motor to stop abruptly using battery power
//      */
//     public void stop() {
//         IntakeMotor.setPercentOutput(0);
//     }


//     /**
//      * Stop command for the intake motor
//      * @return stop command
//      */
//     public Command stopCommand(){
//         return this.runOnce(() -> stop());
//     }


//    /**
//     * Get the velocity of the intake motor in RPM
//     * @return velocity in RPM
//     */
//     public double getVelocity(){
//         return IntakeMotor.getVelocityRPM();
//     }


//     /*
//      * Periodic method, primarily used for logging
//      */
//     @Override
//     public void periodic(){
//         Logger.recordOutput("Intake RPM", getVelocity());
    // }
        
}