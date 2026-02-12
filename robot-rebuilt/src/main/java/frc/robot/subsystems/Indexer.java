package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INDEXER;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.MotorConstants;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;
import frc.robot.helpers.motor.talonfx.TalonFXMotor;

public class Indexer extends SubsystemBase{
    // private KrakenX60Motor spinMotor;
    // private KrakenX60Motor outputMotor;
    //may be 
    private TalonFXMotor spinMotor = new TalonFXMotor(0, false, MotorConstants) {    };
    private TalonFX spinMotor = new TalonFX(INDEXER.SPINNER_CAN_ID);
    private TalonFX outputMotor = new TalonFX(INDEXER.OUTPUT_CAN_ID);

    private PIDProfile gains = new PIDProfile();

    /**
     * Constructor for the Indexer subsystem
     * 
     * Instatiate the motors with initial PID values from the CONSTANTS class
     */
    public Indexer() {
        // spinMotor = new KrakenX60Motor(0, false);
        // spinMotor.setCurrentLimit(0);
        // spinMotor.setIdleMode(IdleMode.kCoast); //the spin can cruise to a stop
        // spinMotor.configureMotionMagic(getVelocitySpinner(), getVelocityOutput());
        
        // outputMotor = new KrakenX60Motor(0, false);
        // outputMotor.setCurrentLimit(0);
        // outputMotor.setIdleMode(IdleMode.kBrake); //the motor must stop pushing out fuel immediately
        // outputMotor.configureMotionMagic(getVelocitySpinner(), getVelocityOutput());

        spinMotor.

        // TODO: For tuning, put the PID and velocity values on the dashboard.
        // SmartDashboard.putNumber("P_SPINNER", INDEXER.SPIN_P);
        // SmartDashboard.putNumber("I_SPINNER", INDEXER.SPIN_I);
        // SmartDashboard.putNumber("D_SPINNER", INDEXER.SPIN_D);

        // SmartDashboard.putNumber("P_OUTPUT", INDEXER.OUTPUT_P);
        // SmartDashboard.putNumber("I_OUTPUT", INDEXER.OUTPUT_I);
        // SmartDashboard.putNumber("D_OUTPUT", INDEXER.OUTPUT_D);
    }

    /**
     * Run the indexer at a set speed from the SmartDashboard
     */
    public void runAtSpeed(){
        double RPM_SPINNER = SmartDashboard.getNumber("SPINNER", INDEXER.SPINNER_VI);
        double RPM_OUTPUT = SmartDashboard.getNumber("Vi_OUTPUT", INDEXER.OUTPUT_VI);
    }

    /**
     * Command to run the indexer at a set speed
     * @return
     */
    public Command runAtSpeedCommand(){
        return this.runOnce(() -> runAtSpeed());
    }

    /**
     * Stop the indexer motors
     * 
     * The spinnner will spin down naturally
     * The output motor must stop immeidately so we do not continue to feed the shooter
     */
    public void stop(){
        spinMotor.setPercentOutput(0);
        outputMotor.setPercentOutput(0);
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
        return spinMotor.getVelocityRPM();
        // return spinnerEncoder.getVelocity();
    }

    /**
     * Get the velocity of the output motor in RPM
     * @return velocity in RPM
     */
    public double getVelocityOutput(){
        return outputMotor.getVelocityRPM();
        // return outputEncoder.getVelocity();
    }

    /**
     * Periodic method, primarily used for logging
     */
    @Override
    public void periodic(){
        // Get motors speeds in RPM
        SmartDashboard.putNumber("Spinner RPM", getVelocitySpinner());
        SmartDashboard.putNumber("Output RPM", getVelocityOutput());
    }
        
}