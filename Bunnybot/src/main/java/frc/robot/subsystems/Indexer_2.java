package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
import frc.robot.helpers.motor.spark.SparkMaxMotor;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;

import frc.robot.Constants.CAN;
import frc.robot.Constants.INDEXER;
import frc.robot.Constants.LAUNCHER;


// Two Motors - 1 constantly running - 1 running at different speeds - Krakens
public class Indexer_2 extends SubsystemBase{
    KrakenX60Motor spinner;

    public Indexer_2() {
        spinner = new KrakenX60Motor(CAN.INDEXER_1_MOTOR_1);

        SmartDashboard.putNumber("indexer_spinner_motor", 0.2); 
 
    }

    /** 
     * Accepts the desired speed as a percentage and sets the motors to the given speed. 
     * @param percent Desired speed as a percentage.
    */
    public void setIndexerPercentOutput (double percent1) {
        spinner.setPercentOutput(percent1);
    }

    /**
     * Stops the indexer motors by setting by setting the percent output to 0
     */
    public void stop() {
        setIndexerPercentOutput(0);
    }

    /**
     * Accepts the desired power of the motors and sets the motors to that power.
     * @param percent Desired percentage of the motors.
     * @return Returns a command to set motor power to given percentage.
     */
    public Command setIndexerCommand(double percent1) {
        return this.runOnce(() -> 
            {setIndexerPercentOutput(percent1);});
    }

    /**
     * Stops the indexer motor by setting the percent output to 0.
     * @return Returns a command to stop the intake motor.
     */
    public Command stopCommand() {
        return this.runOnce(() -> 
            {setIndexerPercentOutput(0);});
    }


}