package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INDEXER;
import frc.robot.helpers.SparkFlexControl;

public class IndexerBetter extends SubsystemBase{
    private SparkFlexControl spinMotor;

    public IndexerBetter(){
        spinMotor = new SparkFlexControl(INDEXER.OUTPUT_CAN_ID, true);
        spinMotor.setInverted();

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
        return this.runOnce(() -> spinMotor.stop());
    }

    public Command runIndexerCommand(){
        return this.runOnce(() -> spinMotor.setVoltage(11.0));
    }

    public Command runReverseIndexerCommand(){
        return this.runOnce(() -> spinMotor.setVoltage(-11.0));
    }
}
