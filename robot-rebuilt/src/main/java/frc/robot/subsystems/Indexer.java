package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INDEXER;
import frc.robot.helpers.PIDProfile;

public class Indexer extends SubsystemBase {    
    //TODO: confirm CAN IDs
    private TalonFX spinMotor = new TalonFX(INDEXER.SPINNER_CAN_ID);
    private TalonFX outputMotor = new TalonFX(INDEXER.OUTPUT_CAN_ID);

    private TalonFXConfiguration spinMotorConfiguration = new TalonFXConfiguration();
    private TalonFXConfiguration outputMotorConfiguration = new TalonFXConfiguration();

    private PIDProfile spinMotorGains = new PIDProfile();
    private PIDProfile outputMotorGains = new PIDProfile();

    /**
     * Constructor for the Indexer subsystem
     * 
     * Instatiate the motors with initial PID values from the CONSTANTS class
     */
    public Indexer() {
        //TODO: consider adding a software stop for rotations if not in the hardware already
        //TODO: change the current limits

        spinMotor.setCurrentLimit(INDEXER.SPIN_CURRENT_LIMIT_FREE);
        spinMotor.setIdleMode(IdleMode.kCoast); //the spin can cruise to a stop
        
        outputMotor.setCurrentLimit(INDEXER.OUTPUT_CURRENT_LIMIT_STALL);
        outputMotor.setIdleMode(IdleMode.kBrake); //the motor must stop pushing out fuel immediately

        // TODO: Tune pid
        SmartDashboard.putNumber("P_SPINNER", INDEXER.SPIN_P);
        SmartDashboard.putNumber("I_SPINNER", INDEXER.SPIN_I);
        SmartDashboard.putNumber("D_SPINNER", INDEXER.SPIN_D);

        SmartDashboard.putNumber("P_OUTPUT", INDEXER.OUTPUT_P);
        SmartDashboard.putNumber("I_OUTPUT", INDEXER.OUTPUT_I);
        SmartDashboard.putNumber("D_OUTPUT", INDEXER.OUTPUT_D);

        spinMotorGains.setPID(INDEXER.SPIN_P, INDEXER.SPIN_I, INDEXER.SPIN_D);
        spinMotorGains.setSlot(0);
        spinMotor.withGains(spinMotorGains);        

        outputMotorGains.setPID(INDEXER.OUTPUT_P, INDEXER.OUTPUT_I, INDEXER.OUTPUT_D);
        outputMotorGains.setSlot(1);
        outputMotor.withGains(outputMotorGains);

    }

    @Override
    public void periodic(){
        // Get motors speeds in RPM
        SmartDashboard.putNumber("Spinner RPM", getSpinnerVelocity());
        SmartDashboard.putNumber("Output RPM", getOutputVelocity());
    }

    /**
     * Run the indexer at a set speed from the SmartDashboard
     */
    // public void runAtSpeed(){
    //     runSpinIndexer();
    //     runOutputIndexer();
    // }

    /**
     * Runs the indexer at a set speed from the SmartDashboard
     * @return a command to run the motors at the desired speed
     */
    // public Command runAtSpeedCommand(){
    //     return this.runOnce(() -> runAtSpeed());
    // }

    /**
     * Stop the indexer motors; 
     * The spinnner will spin down naturally
     * The output motor must stop immeidately so we do not continue to feed the shooter
     */
    public void stop(){
        spinMotor.setVelocity(INDEXER.SPIN_MOTOR_STOP_SPEED, 0);
        outputMotor.setVelocity(INDEXER.OUTPUT_MOTOR_STOP_SPEED, 1);
    }

    /**
     * Command to stop the indexer motors
     * @return a command to stop the indexer motors
     */
    public Command stopCommand(){
        return this.runOnce(() -> stop());
    }

    /**
     * Runs the spin motor on the indexer
     */
    public void runSpinIndexer(){
        spinMotor.setVelocity(INDEXER.SPIN_MOTOR_SPEED, 0);
    }

    /**
     * Command to run the spin motor on the indexer 
     * @return a command to run the spin motor on the indexer
     */
    public Command runSpinIndexerCommand(){
        return this.run(() -> runSpinIndexer());
    }

    /**
     * Runs the output motor on the indexer at given speed
     */
    public void runOutputIndexer(){
        outputMotor.setVelocity(INDEXER.OUTPUT_MOTOR_SPEED, 1);
    }

    /**
     * Command to run the output motor on the indexer 
     * @return a Command to run the output motor on the indexer 
     */
    public Command runOutputIndexerCommand(){
        return this.run(() -> runOutputIndexer());
    }

    /**
     * Runs both motors on the indexer 
     */
    public void runIndexer(){
        spinMotor.setVelocity(INDEXER.SPIN_MOTOR_SPEED, 0);
        outputMotor.setVelocity(INDEXER.OUTPUT_MOTOR_SPEED, 1);
    }

    /**
     * Command to run both motors on the indexer 
     * @return a command to run the motors
     */
    public Command runIndexerCommand(){
        return this.run(() -> runIndexer());
    }

    /**
     * Get the velocity of the spinner motor in RPM
     * @return spinner motor velocity in RPM
     */
    public double getSpinnerVelocity(){
        return spinMotor.getVelocityRPM();
    }

    /**
     * Get the velocity of the output motor in RPM
     * @return output motor velocity in RPM
     */
    public double getOutputVelocity(){
        return outputMotor.getVelocityRPM();
    }
    
        
}