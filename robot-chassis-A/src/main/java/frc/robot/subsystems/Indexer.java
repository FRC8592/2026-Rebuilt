package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INDEXER;
public class Indexer extends SubsystemBase {    
    //TODO: confirm CAN IDs
    private TalonFX spinMotor = new TalonFX(INDEXER.SPINNER_CAN_ID);
    private TalonFX outputMotor = new TalonFX(INDEXER.OUTPUT_CAN_ID);

    private TalonFXConfiguration spinMotorConfiguration = new TalonFXConfiguration();
    private TalonFXConfiguration outputMotorConfiguration = new TalonFXConfiguration();

    private VelocityVoltage spinMotorSlot0VelocityRequest = new VelocityVoltage(0);
    private VelocityVoltage outputMotorSlot0VelocityRequest = new VelocityVoltage(0);

    private double P_OLD;
    private double I_OLD;
    private double D_OLD;

    /**
     * Constructor for the Indexer subsystem
     * 
     * Instatiate the motors with initial PID values from the CONSTANTS class
     */
    public Indexer() {
        //TODO: change the current limits constants
        spinMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        spinMotorConfiguration.CurrentLimits.StatorCurrentLimit = INDEXER.SPIN_CURRENT_LIMIT_FREE;
        spinMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        spinMotor.setNeutralMode(NeutralModeValue.Coast); //the spin can cruise to a stop

        outputMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        outputMotorConfiguration.CurrentLimits.StatorCurrentLimit = INDEXER.OUTPUT_CURRENT_LIMIT_STALL;
        outputMotor.setNeutralMode(NeutralModeValue.Brake); //the motor must stop pushing out fuel immediately

        // TODO: Tune pid
        SmartDashboard.putNumber("P_SPINNER", INDEXER.SPIN_P);
        SmartDashboard.putNumber("I_SPINNER", INDEXER.SPIN_I);
        SmartDashboard.putNumber("D_SPINNER", INDEXER.SPIN_D);
        SmartDashboard.putNumber("VEL_SPINNER", INDEXER.SPIN_MOTOR_SPEED);

        SmartDashboard.putNumber("P_OUTPUT", INDEXER.OUTPUT_P);
        SmartDashboard.putNumber("I_OUTPUT", INDEXER.OUTPUT_I);
        SmartDashboard.putNumber("D_OUTPUT", INDEXER.OUTPUT_D);
        SmartDashboard.putNumber("VEL_OUTPUT", INDEXER.OUTPUT_MOTOR_SPEED);

        spinMotorConfiguration.Slot0.kP  = INDEXER.SPIN_P;
        spinMotorConfiguration.Slot0.kI  = INDEXER.SPIN_I;
        spinMotorConfiguration.Slot0.kD  = INDEXER.SPIN_D;

        outputMotorConfiguration.Slot0.kP = INDEXER.OUTPUT_P;
        outputMotorConfiguration.Slot0.kI = INDEXER.OUTPUT_I;
        outputMotorConfiguration.Slot0.kD = INDEXER.OUTPUT_D;

        //assigns configuration to motor
        spinMotor.getConfigurator().apply(spinMotorConfiguration);
        outputMotor.getConfigurator().apply(outputMotorConfiguration);

    }

    @Override
    public void periodic(){
        // Get motors speeds in RPS
        SmartDashboard.putNumber("Spinner RPS", getSpinnerVelocity());
        SmartDashboard.putNumber("Output RPS", getOutputVelocity());
    }

    /**
     * Stop the indexer motors; 
     * The spinnner will spin down naturally
     * The output motor must stop immeidately so we do not continue to feed the shooter
     */
    public void stop(){
        spinMotor.setControl(spinMotorSlot0VelocityRequest.withSlot(0).withVelocity(0));
        outputMotor.setControl(outputMotorSlot0VelocityRequest.withVelocity(0));
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
        System.out.println("Spindexer Running");
        spinMotor.setControl(spinMotorSlot0VelocityRequest.withSlot(0).withVelocity(SmartDashboard.getNumber("VEL_SPINNER", INDEXER.SPIN_MOTOR_SPEED)));
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
        outputMotor.setVoltage(6);
        //outputMotor.setControl(outputMotorSlot0VelocityRequest.withVelocity(SmartDashboard.getNumber("VEL_OUTPUT", INDEXER.OUTPUT_MOTOR_SPEED)));
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
        runSpinIndexer();
        runOutputIndexer();
    }

    /**
     * Command to run both motors on the indexer 
     * @return a command to run the motors
     */
    public Command runIndexerCommand(){
        return this.run(() -> runIndexer());
    }

    /**
     * Get the velocity of the spinner motor in RPS
     * @return spinner motor velocity in RPS
     */
    public double getSpinnerVelocity(){
        return spinMotor.getVelocity().getValueAsDouble();
    }

    /**
     * Get the velocity of the output motor in RPS
     * @return output motor velocity in RPS
     */
    public double getOutputVelocity(){
        return outputMotor.getVelocity().getValueAsDouble();
    }
    
    public void updatePID(){

        double Spin_P  = SmartDashboard.getNumber("P_SPINNER", INDEXER.SPIN_P);
        double Spin_I  = SmartDashboard.getNumber("I_SPINNER", INDEXER.SPIN_I);
        double Spin_D  = SmartDashboard.getNumber("D_SPINNER", INDEXER.SPIN_D);

        // outputMotorConfiguration.Slot0.kP = SmartDashboard.getNumber("P_OUTPUT", INDEXER.OUTPUT_P);
        // outputMotorConfiguration.Slot0.kI = SmartDashboard.getNumber("I_OUTPUT", INDEXER.OUTPUT_I);
        // outputMotorConfiguration.Slot0.kD = SmartDashboard.getNumber("D_OUTPUT", INDEXER.OUTPUT_D);

        if(Spin_P != P_OLD || Spin_I != I_OLD || Spin_D != D_OLD){
            spinMotorConfiguration.Slot0.kP = Spin_P; 
            spinMotorConfiguration.Slot0.kI = Spin_I;
            spinMotorConfiguration.Slot0.kD = Spin_D; 

            P_OLD = Spin_P;
            I_OLD = Spin_I;
            D_OLD = Spin_D;
            spinMotor.getConfigurator().apply(spinMotorConfiguration);

        }
        
        //outputMotor.getConfigurator().apply(outputMotorConfiguration);
        
    }
}