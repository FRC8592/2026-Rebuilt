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
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.FeedForwardConfig;
import com.revrobotics.spark.config.SparkFlexConfig;

//import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INDEXER;

public class Indexer extends SubsystemBase {    
    private SparkFlex spinMotor = new SparkFlex(INDEXER.SPINNER_CAN_ID, MotorType.kBrushless);
    private SparkFlex outputMotor = new SparkFlex(INDEXER.OUTPUT_CAN_ID, MotorType.kBrushless);

    private FeedForwardConfig spinFeedForward = new FeedForwardConfig();
    private FeedForwardConfig outputFeedForward = new FeedForwardConfig();

    private SparkFlexConfig spinMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig outputMotorConfig = new SparkFlexConfig();

    private SparkClosedLoopController spinMotorClosedLoopCtrl =  spinMotor.getClosedLoopController();
    private SparkClosedLoopController outputMotorClosedLoopCtrl = outputMotor.getClosedLoopController();

    private RelativeEncoder spinMotorEncoder = spinMotor.getEncoder();
    private RelativeEncoder outputMotorEncoder = outputMotor.getEncoder();

    private double PS_OLD;
    private double IS_OLD;
    private double DS_OLD;
    private double SS_OLD;

    private double PO_OLD;
    private double IO_OLD;
    private double DO_OLD;

    /**
     * Constructor for the Indexer subsystem
     * 
     * Instatiate the motors with initial PID values from the CONSTANTS class
     */
    public Indexer() {
        //TODO: Change the current limits constants
        spinMotorConfig.smartCurrentLimit(INDEXER.SPIN_CURRENT_LIMIT);
        spinMotorConfig.inverted(true);     // Sets the motor to to make clockwise rotation positive
        spinMotorConfig.idleMode(IdleMode.kCoast);

        // TODO: Change the current limits constants
        outputMotorConfig.smartCurrentLimit(INDEXER.OUTPUT_CURRENT_LIMIT);
        outputMotorConfig.idleMode(IdleMode.kCoast); 
        
        // TODO: Tune pid
        // SmartDashboard.putNumber("P_SPINNER", INDEXER.SPIN_P);
        // SmartDashboard.putNumber("I_SPINNER", INDEXER.SPIN_I);
        // SmartDashboard.putNumber("D_SPINNER", INDEXER.SPIN_D);
        // SmartDashboard.putNumber("S_SPINNER", INDEXER.SPIN_S);
        // SmartDashboard.putNumber("VEL_SPINNER", INDEXER.SPIN_MOTOR_SPEED);

        // SmartDashboard.putNumber("P_OUTPUT", INDEXER.OUTPUT_P);
        // SmartDashboard.putNumber("I_OUTPUT", INDEXER.OUTPUT_I);
        // SmartDashboard.putNumber("D_OUTPUT", INDEXER.OUTPUT_D);
        // SmartDashboard.putNumber("VEL_OUTPUT", INDEXER.OUTPUT_MOTOR_SPEED);

        spinMotorConfig.closedLoop.pid(INDEXER.SPIN_P, INDEXER.SPIN_I, INDEXER.SPIN_D, ClosedLoopSlot.kSlot0);
        // TODO: kS term might not work, need to check
        spinFeedForward.kS(INDEXER.SPIN_S, ClosedLoopSlot.kSlot0);
        spinMotorConfig.closedLoop.apply(spinFeedForward);

        outputMotorConfig.closedLoop.pid(INDEXER.OUTPUT_P, INDEXER.OUTPUT_I, INDEXER.OUTPUT_D);

        //assigns configuration to motor
        spinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        outputMotor.configure(outputMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    @Override
    public void periodic(){
        // Get motors speeds in RPM
        Logger.recordOutput("Spinner RPM", getSpinnerVelocity());
        Logger.recordOutput("Output RPM", getOutputVelocity());
    }


    /**
     * Stop the indexer motors.  Use brake mode and not motor power to stop
     */
    public void stop(){
        spinMotor.setVoltage(0);
        outputMotor.setVoltage(0);
    }


    /**
     * Runs the spin motor on the indexer
     */
    public void runSpinner(){
        //spinMotorClosedLoopCtrl.setSetpoint(SmartDashboard.getNumber("VEL_SPINNER", INDEXER.SPIN_MOTOR_SPEED), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        // TODO: Revert to closed loop control after testing
        spinMotor.setVoltage(6.0);
    }


    /**
     * Runs the output motor on the indexer at given speed
     */
    public void runOutput(){
        outputMotor.setVoltage(11.0);
        // outputMotorClosedLoopCtrl.setSetpoint(SmartDashboard.getNumber("VEL_OUTPUT", INDEXER.OUTPUT_MOTOR_SPEED), ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }


    /**
     * Runs both the spinner and the output motors on the indexer 
     */
    public void runIndexer(){
        runSpinner();
        runOutput();
    }
    

    /**
     * Command to stop the indexer motors
     * @return a command to stop the indexer motors
     */
    public Command stopCommand(){
        return this.runOnce(() -> stop());
    }


    /**
     * Command to run the spin motor on the indexer 
     * @return a command to run the spin motor on the indexer
     */
    public Command runSpinnerCommand(){
        return this.runOnce(() -> runSpinner());
    }


    /**
     * Command to run the output motor on the indexer 
     * @return a Command to run the output motor on the indexer 
     */
    public Command runOutputCommand(){
        return this.runOnce(() -> runOutput());
    }


    /**
     * Command to run both motors on the indexer 
     * @return a command to run the motors
     */
    public Command runIndexerCommand(){
        return this.runOnce(() -> runIndexer());
    }


    /**
     * Get the velocity of the spinner motor in RPM
     * @return spinner motor velocity in RPM
     */
    public double getSpinnerVelocity(){
        return spinMotorEncoder.getVelocity();
    }


    /**
     * Get the velocity of the output motor in RPM
     * @return output motor velocity in RPM
     */
    public double getOutputVelocity(){
        return outputMotorEncoder.getVelocity();
    }
    
    // public void updatePID(){
    //     double Spin_P = SmartDashboard.getNumber("P_SPINNER", INDEXER.SPIN_P);
    //     double Spin_I = SmartDashboard.getNumber("I_SPINNER", INDEXER.SPIN_I);
    //     double Spin_D = SmartDashboard.getNumber("D_SPINNER", INDEXER.SPIN_D);
    //     double Spin_S = SmartDashboard.getNumber("S_SPINNER", INDEXER.SPIN_S);

    //     double Output_P = SmartDashboard.getNumber("P_OUTPUT", INDEXER.OUTPUT_P);
    //     double Output_I = SmartDashboard.getNumber("I_OUTPUT", INDEXER.OUTPUT_I);
    //     double Output_D = SmartDashboard.getNumber("D_OUTPUT", INDEXER.OUTPUT_D);

    //     if(Spin_P != PS_OLD || Spin_I != IS_OLD || Spin_D != DS_OLD || Spin_S != SS_OLD || Output_P != PO_OLD || Output_I != IO_OLD || Output_D != DO_OLD){

    //         spinMotorConfig.closedLoop.pid(Spin_P, Spin_I, Spin_D, ClosedLoopSlot.kSlot0);
    //         spinFeedForward.kS(Spin_S, ClosedLoopSlot.kSlot0);
    //         spinMotorConfig.closedLoop.apply(spinFeedForward);

    //         outputMotorConfig.closedLoop.pid(Output_P, Output_I, Output_D, ClosedLoopSlot.kSlot0);

    //         PS_OLD = Spin_P;
    //         IS_OLD = Spin_I;
    //         DS_OLD = Spin_D;
    //         SS_OLD = Spin_S;

    //         PO_OLD = Output_P;
    //         IO_OLD = Output_I;
    //         DO_OLD = Output_D;

    //         spinMotor.configure(spinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    //         outputMotor.configure(outputMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    //     }
    // }
}