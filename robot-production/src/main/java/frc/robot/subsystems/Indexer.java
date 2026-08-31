package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INDEXER;
import frc.robot.helpers.SparkFlexControl;

public class Indexer extends SubsystemBase{
    private SparkFlexControl spinMotor;

    public boolean indexerRunning;

    public Indexer(){
        spinMotor = new SparkFlexControl(INDEXER.OUTPUT_CAN_ID, true);
        spinMotor.setInverted();
        spinMotor.setCurrentLimit(INDEXER.SPIN_CURRENT_LIMIT);

        // To Tune PID
        // SmartDashboard.putNumber("P_SPINNER", INDEXER.SPIN_P);
        // SmartDashboard.putNumber("I_SPINNER", INDEXER.SPIN_I);
        // SmartDashboard.putNumber("D_SPINNER", INDEXER.SPIN_D);
        // SmartDashboard.putNumber("S_SPINNER", INDEXER.SPIN_S);
        // SmartDashboard.putNumber("VEL_SPINNER", INDEXER.SPIN_MOTOR_SPEED);

        spinMotor.setPIDF(INDEXER.SPIN_P, 
        INDEXER.SPIN_I, 
        INDEXER.SPIN_D, 
        INDEXER.SPIN_S, 
        INDEXER.SPIN_V, 
        INDEXER.SPIN_A, 
        INDEXER.SPIN_G, 
        INDEXER.SPIN_ALLOWED_ERROR);
    }

    @Override
    public void periodic(){
        Logger.recordOutput(INDEXER.LOG_PATH + "Spinner RPM", spinMotor.getVelocity());
        Logger.recordOutput(INDEXER.LOG_PATH + "Spinner Current", spinMotor.getCurrent());

    }

    public Command stopCommand(){
        return this.runOnce(() -> spinMotor.stop()).andThen(() -> {indexerRunning = false;});
    }

    public Command runIndexerCommand(){
        return this.runOnce(() -> spinMotor.setVoltage(11.0)).andThen(() -> {indexerRunning = true;});
    }

    public Command runReverseIndexerCommand(){
        return this.runOnce(() -> spinMotor.setVoltage(-11.0)).andThen(() -> {indexerRunning = true;});
    }
}
