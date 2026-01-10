package frc.robot.subsystems;

//import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.NewtonMotor.IdleMode;
// import frc.robot.helpers.motor.spark.*;
import frc.robot.helpers.motor.talonfx.Falcon500Motor;

//TODO: Change the variables names based on the location of the motor. Top and Bottom motor based on the placement at the laucher. 

public class Launcher extends SubsystemBase {
    // Declaring motors used for launcher
    private NewtonMotor bottomLauncherMotor; 
    private NewtonMotor topLauncherMotor; 
    public Launcher() {
        //Top is Motor2
        //Bottom is Motor1
        bottomLauncherMotor = new Falcon500Motor(CAN.BOTTOM_LAUNCHER_MOTOR, true);
        topLauncherMotor = new Falcon500Motor(CAN.TOP_LAUNCHER_MOTOR, true);
        SmartDashboard.putNumber("bottom_launcher_motor", 0.44); // High: 0.4 Low: 0.23  Close shot: 0.44
        SmartDashboard.putNumber("top_launcher_motor", 0.3); // High: 0.4 Low: 0.23   Close shot: 0.3

        //launcherSensor = new LaserCan(CAN.LAUNCHER2_BEAM_BREAK_CAN_ID);

        bottomLauncherMotor.setIdleMode(IdleMode.kCoast);
        topLauncherMotor.setIdleMode(IdleMode.kCoast);

    }

    /** 
     * Accepts the desired speed as a percentage and sets the motor to the given speed. 
     * @param percent Desired speed as a percentage.
    */
    public void setLauncherPercentOutput(double percent1, double percent2){
        //Debugging to try and see whether the method is running, which it does
        System.out.println("Set Launch % Output called: " + percent1 + " " + percent2);
        bottomLauncherMotor.setPercentOutput(percent1);
        topLauncherMotor.setPercentOutput(percent2);

    }

    /**
     * Stops the launcher motor by setting by setting the percent output to 0
     */
    public void stop(){
        setLauncherPercentOutput(0,0);
    }
    /**
     * Accepts the desired power of the motor and sets the motor to that power.
     * @param percent Desired percentage of the motor.
     * @return Returns a command to set motor power to given percentage.
     */
    public Command setLauncherCommand(double percent1, double percent2){
        return this.run(()->{setLauncherPercentOutput(percent1, percent2);
        });
    }

    /**
     * Stops the intake motor by setting the percent output to 0.
     * @return Returns a command to stop the intake motor.
     */
    public Command stopLauncherCommand(){
        return this.runOnce(()->{
            setLauncherPercentOutput(0,0);
        });
    }

}
