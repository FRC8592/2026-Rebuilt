package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INDEXER;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.MotorConstants;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.talonfx.TalonFXMotor;

public class Indexer extends SubsystemBase{    
    //TODO: must add the numbers for motor constants
    MotorConstants spinMotorConstants = new MotorConstants(0, 0, 0, 0);
    MotorConstants outputMotorConstants = new MotorConstants(0, 0, 0, 0);
    //TODO: confirm CAN IDs
    private TalonFXMotor spinMotor = new TalonFXMotor(INDEXER.SPINNER_CAN_ID, spinMotorConstants) {};
    private TalonFXMotor outputMotor = new TalonFXMotor(INDEXER.OUTPUT_CAN_ID, outputMotorConstants) {};

    private PIDProfile spinMotorGains = new PIDProfile();
    private PIDProfile outputMotorGains = new PIDProfile();

    /**
     * Constructor for the Indexer subsystem
     * 
     * Instatiate the motors with initial PID values from the CONSTANTS class
     */
    public Indexer() {
        //TODO: consider adding a software stop for rotations if not in the hardware already
        spinMotor.setCurrentLimit(INDEXER.SPIN_CURRENT_LIMIT_FREE);
        spinMotor.setIdleMode(IdleMode.kCoast); //the spin can cruise to a stop
        
        outputMotor.setCurrentLimit(INDEXER.OUTPUT_CURRENT_LIMIT_STALL);
        outputMotor.setIdleMode(IdleMode.kBrake); //the motor must stop pushing out fuel immediately

        // TODO: Tune pid with logging (?)
        // SmartDashboard.putNumber("P_SPINNER", INDEXER.SPIN_P);
        // SmartDashboard.putNumber("I_SPINNER", INDEXER.SPIN_I);
        // SmartDashboard.putNumber("D_SPINNER", INDEXER.SPIN_D);

        // SmartDashboard.putNumber("P_OUTPUT", INDEXER.OUTPUT_P);
        // SmartDashboard.putNumber("I_OUTPUT", INDEXER.OUTPUT_I);
        // SmartDashboard.putNumber("D_OUTPUT", INDEXER.OUTPUT_D);

        spinMotorGains.setPID(INDEXER.SPIN_P, INDEXER.SPIN_I, INDEXER.SPIN_D);
        spinMotor.withGains(spinMotorGains);

        outputMotorGains.setPID(INDEXER.OUTPUT_P, INDEXER.OUTPUT_I, INDEXER.OUTPUT_D);
        outputMotor.withGains(outputMotorGains);

    }

    @Override
    public void periodic(){
        runAtSpeed();

        // Get motors speeds in RPM
        SmartDashboard.putNumber("Spinner RPM", getSpinnerVelocity());
        SmartDashboard.putNumber("Output RPM", getOutputVelocity());
    }

    /**
     * Run the indexer at a set speed from the SmartDashboard
     */
    public void runAtSpeed(){
        double spinMotorSpeed = SmartDashboard.getNumber("Spin Motor Speed", 0);
        double outputMotorSpeed = SmartDashboard.getNumber("Output Motor Speed", 0);

        runSpinIndexer(spinMotorSpeed);
        runOutputIndexer(outputMotorSpeed);
    }

    /**
     * Runs the indexer at a set speed from the SmartDashboard
     * @return a command to run the motors at the desired speed
     */
    public Command runAtSpeedCommand(){
        return this.runOnce(() -> runAtSpeed());
    }

    /**
     * Stop the indexer motors; 
     * The spinnner will spin down naturally
     * The output motor must stop immeidately so we do not continue to feed the shooter
     */
    public void stop(){
        spinMotor.setPercentOutput(0);
        outputMotor.setPercentOutput(0);
    }

    /**
     * Command to stop the indexer motors
     * @return a command to stop the indexer motors
     */
    public Command stopCommand(){
        return this.runOnce(() -> stop());
    }

    /**
     * Runs the spin motor on the indexer at given speed
     * @param speed new motor speed
     */
    public void runSpinIndexer(double speed){
        spinMotor.setPercentOutput(speed);
    }

    /**
     * Command to run the spin motor on the indexer at the provided speed
     * @param speed new motor speed
     * @return a command to run the spin motor on the indexer at given speed
     */
    public Command runSpinIndexerCommand(double speed){
        return this.run(() -> runSpinIndexer(speed));
    }

    /**
     * Runs the output motor on the indexer at given speed
     * @param speed new motor speed
     */
    public void runOutputIndexer(double speed){
        outputMotor.setPercentOutput(speed);
    }

    /**
     * Command to run the output motor on the indexer at the provided speed
     * @param speed new motor speed
     * @return a Command to run the output motor on the indexer at given speed
     */
    public Command runOutputIndexerCommand(double speed){
        return this.run(() -> runOutputIndexer(speed));
    }

    /**
     * Runs both motors on the indexer at the given speed
     * @param speed new motor speed
     */
    public void runIndexer(double speed){
        spinMotor.setPercentOutput(speed);
        outputMotor.setPercentOutput(speed);
    }

    /**
     * Command to run both motors on the indexer at the given speed
     * @param speed new motor speed
     * @return a command to run the motors
     */
    public Command runIndexerCommand(double speed){
        return this.run(() -> runIndexer(speed));
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