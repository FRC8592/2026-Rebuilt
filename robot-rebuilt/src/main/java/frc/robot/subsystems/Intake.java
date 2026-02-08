package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.ResetMode;
import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.Constants.INTAKE;


public class Intake extends SubsystemBase{
//     private SparkFlex intakeMotor;
//     private SparkFlexConfig intakeConfig;
//     private SparkClosedLoopController intakeController;
//     private RelativeEncoder intakeEncoder;
 
//     /**
//      * Constructor for the Intake subsystem
//      * 
//      * Instatiate the motor with initial PID values from the CONSTANTS class
//      */
//     public Intake() {

//         /*
//          * Create the Intake motor and instatiate the following features
//          *   Reset to safe factory configuration
//          *   Store persistant configuration (Flash)
//          *   Place in COAST mode (Can coast to a stop)
//          *   Set current limits
//          *   Set VELOCITY PID parameters
//          */
//         intakeMotor = new SparkFlex(INTAKE.INTAKE_MOTOR_CAN_ID, MotorType.kBrushless);
//         intakeConfig = new SparkFlexConfig();
//         intakeConfig.idleMode(IdleMode.kCoast);
//         intakeConfig.smartCurrentLimit(INTAKE.INTAKE_CURRENT_LIMIT_STALL, INTAKE.INTAKE_CURRENT_LIMIT_FREE);    // TODO: Set appropriate current limits
//         intakeConfig.closedLoop.pid(INTAKE.INTAKE_P, INTAKE.INTAKE_I, INTAKE.INTAKE_D);                         // TODO: Tune PID gains
        
//         intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         intakeController = intakeMotor.getClosedLoopController();
//         intakeEncoder = intakeMotor.getEncoder();

//         // TODO: For tuning, put the PID and velocity values on the dashboard.  Remove before competition
//         SmartDashboard.putNumber("P_INTAKE", INTAKE.INTAKE_P);
//         SmartDashboard.putNumber("I_INTAKE", INTAKE.INTAKE_I);
//         SmartDashboard.putNumber("D_INTAKE", INTAKE.INTAKE_D);
//         SmartDashboard.putNumber("Vi_INTAKE",INTAKE.INTAKE_VI);
//     }


//     /**
//      * Run the intake at a set speed
//      */
//     public void runAtSpeed() {
//         double RPM = SmartDashboard.getNumber("Vi_INTAKE", INTAKE.INTAKE_VI); // TODO: Remove this before competition
//         intakeController.setSetpoint(RPM, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
//     }


//     /**
//      * Command to run the intake at a set speed
//      */
//     public Command runAtSpeedCommand() {
//         return this.runOnce(() -> runAtSpeed());
//     }


//     /**
//      * Stop the intake motor
//      * 
//      * We do this using voltage mode so that the motor will slow to a stop naturally
//      * Using setVelocity() will cause the motor to stop abruptly using battery power
//      */
//     public void stop() {
//         intakeMotor.setVoltage(0.0);
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
//         return intakeEncoder.getVelocity();
//     }


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
    
//         intakeConfig.closedLoop.pid(P, I, D);
//         intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
//     }


//     /*
//      * Periodic method, primarily used for logging
//      */
//     @Override
//     public void periodic(){
//         Logger.recordOutput("Intake RPM", getVelocity());
//     }
        
}