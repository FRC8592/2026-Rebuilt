package frc.robot.subsystems;

import frc.robot.helpers.motor.talonfx.KrakenX44Motor;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;

public class Turret extends SubsystemBase{
    private KrakenX44Motor tMotor;

    public Turret(){
        tMotor = new KrakenX44Motor(TURRET.TURRET_MOTOR, true);
        PIDProfile gains = new PIDProfile();
        gains.setPID(TURRET.TURRET_P, TURRET.TURRET_I, TURRET.TURRET_D);
        tMotor.withGains(gains);
        //TODO: Implement later
       tMotor.configureMotionMagic(10, 2);
    }
    public void TurrettoPos(double position){
        System.out.println("Going into positioning method");
        tMotor.setPosition(position);
    }
    public void stop(){
        tMotor.setPercentOutput(0);
    }

    public Command TurrettoPosCommand(double position){
        System.out.println("Going into positioning command");
        return this.runOnce(() -> TurrettoPos(position));
    }

    public Command stopTurretCommand(){
        return this.runOnce(() -> stop());
    }
}
