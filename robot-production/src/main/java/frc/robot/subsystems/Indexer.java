package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INDEXER;

public class Indexer extends SubsystemBase {
    private SparkFlex spinMotor = new SparkFlex(INDEXER.SPINNER_CAN_ID, MotorType.kBrushless);

    private FeedForwardConfig spinFeedForward = new FeedForwardConfig();

    private SparkFlexConfig spinMotorConfig = new SparkFlexConfig();

    private SparkClosedLoopController spinMotorClosedLoopCtrl = spinMotor.getClosedLoopController();

    private RelativeEncoder spinMotorEncoder = spinMotor.getEncoder();

    private double PS_OLD;
    private double IS_OLD;
    private double DS_OLD;


    public boolean indexerRunning = false;


    /**
     * Constructor for the Indexer subsystem
     * 
     * Instatiate the motors with initial PID values from the CONSTANTS class
     */
    public Indexer() {
        spinMotorConfig.smartCurrentLimit(INDEXER.SPIN_CURRENT_LIMIT);
        spinMotorConfig.inverted(true); // Sets the motor to to make clockwise rotation positive
        spinMotorConfig.idleMode(IdleMode.kCoast);


        // TODO: Tune pid
        SmartDashboard.putNumber("P_SPINNER", INDEXER.SPIN_P);
        SmartDashboard.putNumber("I_SPINNER", INDEXER.SPIN_I);
        SmartDashboard.putNumber("D_SPINNER", INDEXER.SPIN_D);
        SmartDashboard.putNumber("VEL_SPINNER", INDEXER.SPIN_MOTOR_SPEED);


        spinMotorConfig.closedLoop.pid(INDEXER.SPIN_P, INDEXER.SPIN_I, INDEXER.SPIN_D,
                ClosedLoopSlot.kSlot0);
        // TODO: kS term might not work, need to check
        spinFeedForward.kS(INDEXER.SPIN_S, ClosedLoopSlot.kSlot0);
        spinMotorConfig.closedLoop.apply(spinFeedForward);


        // assigns configuration to motor
        spinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    @Override
    public void periodic() {
        // Get motors speeds in RPM
        Logger.recordOutput(INDEXER.LOG_PATH + "Spinner RPM", getSpinnerVelocity());
        Logger.recordOutput(INDEXER.LOG_PATH + "Spinner Current", getSpinnerCurrent());
    }

    /**
     * Stop the indexer motors. Use brake mode and not motor power to stop
     */
    //TODO: No need for a stop method anymore
    public void stop() {
        stopSpinner();
        indexerRunning = false;
    }

    public void stopSpinner(){
        spinMotor.setVoltage(0d);
    }

    /**
     * Runs the spin motor on the indexer
     */
    public void runSpinner() {
        spinMotor.setVoltage(11.0);
    }


    /**
     * Runs both the spinner and the output motors on the indexer
     */
    public void runIndexer() {
        runSpinner();
        indexerRunning = true;

    }
    


    /**
     * Command to stop the indexer motors
     * 
     * @return a command to stop the indexer motors
     */
    public Command stopCommand() {
        return this.runOnce(() -> stop());
    }

    /**
     * Command to run the spin motor on the indexer
     * 
     * @return a command to run the spin motor on the indexer
     */


    /**
     * Command to run both motors on the indexer
     * 
     * @return a command to run the motors
     */
    public Command runIndexerCommand() {
        return this.runOnce(() -> runIndexer());
    }

    /**
     * Get the velocity of the spinner motor in RPM
     * 
     * @return spinner motor velocity in RPM
     */
    public double getSpinnerVelocity() {
        return spinMotorEncoder.getVelocity();
    }



    public double getSpinnerCurrent(){
        return spinMotor.getOutputCurrent();
    }



    public void updatePID(){
        double Spin_P = SmartDashboard.getNumber("P_SPINNER", INDEXER.SPIN_P);
        double Spin_I = SmartDashboard.getNumber("I_SPINNER", INDEXER.SPIN_I);
        double Spin_D = SmartDashboard.getNumber("D_SPINNER", INDEXER.SPIN_D);


        if (Spin_P != PS_OLD || Spin_I != IS_OLD || Spin_D != DS_OLD) {

            spinMotorConfig.closedLoop.pid(Spin_P, Spin_I, Spin_D, ClosedLoopSlot.kSlot0);

            PS_OLD = Spin_P;
            IS_OLD = Spin_I;
            DS_OLD = Spin_D;

            spinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kNoPersistParameters);
        }
    }
}
