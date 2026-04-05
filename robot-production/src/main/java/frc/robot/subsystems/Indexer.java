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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
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
    private double SS_OLD;

    public boolean indexerRunning = false;

    private ParallelCommandGroup waitandShoot = new ParallelCommandGroup();

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
        SmartDashboard.putNumber("S_SPINNER", INDEXER.SPIN_S);
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
        Logger.recordOutput(INDEXER.LOG_PATH + "Auto Shoot and Stop Finished", waitandShoot.isFinished());
    }

    /**
     * Stop the indexer motor. Use brake mode and not motor power to stop
     */
    public void stop() {
        spinMotor.setVoltage(0);
        indexerRunning = false;
    }

    /**
     * Command to stop the indexer motor
     * 
     * @return a command to stop the indexer motor
     */
    public Command stopCommand() {
        return this.runOnce(() -> stop());
    }

    /**
     * Runs the indexer
     */
    public void runIndexer() {
        spinMotor.setVoltage(11.0);
        indexerRunning = true;
    }

    /**
     * Command to run the indexer
     * 
     * @return a command to run the motor
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

    /**
     * Get the current of the spinner motor in amps
     * 
     * @return spinner motor current in amps
     */
    public double getSpinnerCurrent(){
        return spinMotor.getOutputCurrent();
    }

    public Command waitandShootCommand(){
        waitandShoot = new ParallelCommandGroup(runIndexerCommand(), Commands.waitSeconds(3.0));
        return waitandShoot;
    }

    public void updatePID(){
        double Spin_P = SmartDashboard.getNumber("P_SPINNER", INDEXER.SPIN_P);
        double Spin_I = SmartDashboard.getNumber("I_SPINNER", INDEXER.SPIN_I);
        double Spin_D = SmartDashboard.getNumber("D_SPINNER", INDEXER.SPIN_D);
        double Spin_S = SmartDashboard.getNumber("S_SPINNER", INDEXER.SPIN_S);

        if (Spin_P != PS_OLD || Spin_I != IS_OLD || Spin_D != DS_OLD || Spin_S != SS_OLD) {

            spinMotorConfig.closedLoop.pid(Spin_P, Spin_I, Spin_D, ClosedLoopSlot.kSlot0);
            spinFeedForward.kS(Spin_S, ClosedLoopSlot.kSlot0);
            spinMotorConfig.closedLoop.apply(spinFeedForward);

            PS_OLD = Spin_P;
            IS_OLD = Spin_I;
            DS_OLD = Spin_D;
            SS_OLD = Spin_S;

            spinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters,
                    PersistMode.kNoPersistParameters);
        }
    }
}
