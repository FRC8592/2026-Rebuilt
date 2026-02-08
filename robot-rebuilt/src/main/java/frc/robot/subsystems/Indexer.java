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
import frc.robot.Constants.INDEXER;


public class Indexer extends SubsystemBase{
//     private SparkFlex spinnerMotor;
//     private SparkFlexConfig spinnerConfig;
//     private SparkClosedLoopController spinnerController;
//     private RelativeEncoder spinnerEncoder;
    
//     private SparkFlex outputMotor;
//     private SparkFlexConfig outputConfig;
//     private SparkClosedLoopController outputController;
//     private RelativeEncoder outputEncoder;

//  //   private PIDProfile MotorPID; 

//     /**
//      * Constructor for the Indexer subsystem
//      * 
//      * Instatiate the motors with initial PID values from the CONSTANTS class
//      */
//     public Indexer() {

//         /*
//          * Create the Spinner motor and instatiate the following features
//          *   Reset to safe factory configuration
//          *   Store persistant configuration (Flash)
//          *   Place in COAST mode (Can coast to a stop)
//          *   Set current limits
//          *   Set VELOCITY PID parameters
//          */
//         spinnerMotor = new SparkFlex(INDEXER.SPINNER_CAN_ID, MotorType.kBrushless);
//         spinnerConfig = new SparkFlexConfig();
//         spinnerConfig.idleMode(IdleMode.kCoast);
//         spinnerConfig.smartCurrentLimit(INDEXER.SPINNER_CURRENT_LIMIT_STALL, INDEXER.SPINNER_CURRENT_LIMIT_FREE);   // TODO: Set appropriate current limits
//         spinnerConfig.closedLoop.pid(INDEXER.SPINNER_P, INDEXER.SPINNER_I, INDEXER.SPINNER_D);                      // TODO: Tune PID gains
                            
//         spinnerMotor.configure(spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         spinnerController = spinnerMotor.getClosedLoopController();
//         spinnerEncoder = spinnerMotor.getEncoder();
        
//         /*
//          * Create the Output motor and instatiate the following features
//          *   Reset to safe factory configuration
//          *   Store persistant configuration (Flash)
//          *   Place in BRAKE mode (stop feeding fuel to the shooter quickly)
//          *   Set current limits
//          *   Set VELOCITY PID parameters
//          */
//         outputMotor = new SparkFlex(INDEXER.OUTPUT_CAN_ID, MotorType.kBrushless);
//         outputConfig = new SparkFlexConfig();
//         outputConfig.idleMode(IdleMode.kBrake);
//         outputConfig.smartCurrentLimit(INDEXER.OUTPUT_CURRENT_LIMIT_STALL, INDEXER.OUTPUT_CURRENT_LIMIT_FREE);  // TODO: Set appropriate current limits
//         outputConfig.closedLoop.pid(INDEXER.OUTPUT_P, INDEXER.OUTPUT_I, INDEXER.OUTPUT_D);                      // TODO: Tune PID gains

//         outputMotor.configure(outputConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
//         outputController = outputMotor.getClosedLoopController();
//         outputEncoder = outputMotor.getEncoder();

//         // TODO: For tuning, put the PID and velocity values on the dashboard.  Remove before competition
//         SmartDashboard.putNumber("P_SPINNER", INDEXER.SPINNER_P);
//         SmartDashboard.putNumber("I_SPINNER", INDEXER.SPINNER_I);
//         SmartDashboard.putNumber("D_SPINNER", INDEXER.SPINNER_D);
//         SmartDashboard.putNumber("Vi_SPINNER", INDEXER.SPINNER_VI);

//         SmartDashboard.putNumber("P_OUTPUT", INDEXER.OUTPUT_P);
//         SmartDashboard.putNumber("I_OUTPUT", INDEXER.OUTPUT_I);
//         SmartDashboard.putNumber("D_OUTPUT", INDEXER.OUTPUT_D);
//         SmartDashboard.putNumber("Vi_OUTPUT", INDEXER.OUTPUT_VI);
//     }


//     /**
//      * Run the indexer at a set speed
//      */
//     public void runAtSpeed(){
//         double RPM_SPINNER = SmartDashboard.getNumber("Vi_SPINNER", INDEXER.SPINNER_VI);
//         double RPM_OUTPUT = SmartDashboard.getNumber("Vi_OUTPUT", INDEXER.OUTPUT_VI);

//         // Apply closed loop controls to the Controller, not directly to the motor
//         spinnerController.setSetpoint(RPM_SPINNER, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
//         // TODO: Put output motor into velocity control
//         //outputController.setSetpoint(RPM_OUTPUT, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
//         outputMotor.setVoltage(13); // Run at full power
//     }


//     /**
//      * Command to run the indexer at a set speed
//      * @return
//      */
//     public Command runAtSpeedCommand(){
//         return this.runOnce(() -> runAtSpeed());
//     }


//     /**
//      * Stop the indexer motors
//      * 
//      * The spinnner can spin down naturally
//      * The output motor must stop immeidately so we do not continue to feed the shooter
//      */
//     public void stop(){
//         spinnerMotor.setVoltage(0.0);
//         outputMotor.setVoltage(0.0); // In brake mode, should stop quickly
//     }


//     /**
//      * Stop command for the indexer motors
//      * @return stop command
//      */
//     public Command stopCommand(){
//         return this.runOnce(() -> stop());
//     }


//     /**
//      * Get the velocity of the spinner motor in RPM
//      * @return velocity in RPM
//      */
//     public double getVelocitySpinner(){
//         return spinnerEncoder.getVelocity();
//     }

//     /**
//      * Get the velocity of the output motor in RPM
//      * @return velocity in RPM
//      */
//     public double getVelocityOutput(){
//         return outputEncoder.getVelocity();
//     }


// /**
//      * Update the PID constants for the indexer motors from SmartDashboard values
//      * 
//      * The Neo Vortex motors will not accept a change to the PID parameters while running.
//      * Thusly, this method must be called from disabledPeriod() in Robot.java.
//      */
//     public void updatePID(){
//         //System.out.println("Going into updatePID Method");
//         double P_SPINNER = SmartDashboard.getNumber("P_SPINNER", INDEXER.SPINNER_P);
//         double I_SPINNER = SmartDashboard.getNumber("I_SPINNER", INDEXER.SPINNER_I);
//         double D_SPINNER = SmartDashboard.getNumber("D_SPINNER", INDEXER.SPINNER_D);

//         spinnerConfig.closedLoop.pid(P_SPINNER, I_SPINNER, D_SPINNER);
//         spinnerMotor.configure(spinnerConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

//         double P_OUTPUT = SmartDashboard.getNumber("P_OUTPUT", INDEXER.OUTPUT_P);
//         double I_OUTPUT = SmartDashboard.getNumber("I_OUTPUT", INDEXER.OUTPUT_I);
//         double D_OUTPUT = SmartDashboard.getNumber("D_OUTPUT", INDEXER.OUTPUT_D);

//         outputConfig.closedLoop.pid(P_OUTPUT, I_OUTPUT, D_OUTPUT);
//         outputMotor.configure(outputConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);    
//     }


//     /**
//      * Periodic method, primarily used for logging
//      */
//     @Override
//     public void periodic(){
//         // Get motors speeds in RPM
//         SmartDashboard.putNumber("Spinner RPM", getVelocitySpinner());
//         SmartDashboard.putNumber("Output RPM", getVelocityOutput());
//     }
        
}