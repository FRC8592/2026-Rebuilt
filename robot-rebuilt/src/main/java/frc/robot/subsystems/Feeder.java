package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.spark.SparkFlexMotor;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;


public class Feeder extends SubsystemBase{
    private KrakenX60Motor FeederMotor; 
 
    /**
     * Constructor for the Intake subsystem
     * 
     * Instatiate the motor with initial PID values from the CONSTANTS class
     */
    public Feeder() {
        FeederMotor = new KrakenX60Motor(14, false);

    }


    /**
     * Run the intake at a set speed
     */
    public void runAtSpeed() {
        FeederMotor.setPercentOutput(0.2);
    }


    /**
     * Command to run the intake at a set speed
     */
    public Command runAtSpeedCommand() {
        return this.runOnce(() -> runAtSpeed());
    }


    

    /**
     * Stop the intake motor
     * 
     * We do this using % ouptput so that the motor will slow to a stop naturally
     * Using setVelocity() will cause the motor to stop abruptly using battery power
     */
    public void stop() {
        FeederMotor.setPercentOutput(0);
    }


    /**
     * Stop command for the intake motor
     * @return stop command
     */
    public Command stopCommand(){
        return this.runOnce(() -> stop());
    }

        
}
