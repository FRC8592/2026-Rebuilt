package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

 
import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;


public class Shooter extends SubsystemBase{
    private KrakenX60Motor LeftShooterMotor;
    private KrakenX60Motor RightShooterMotor;
    private PIDProfile MotorPID;

    public Shooter(){
        LeftShooterMotor = new KrakenX60Motor(SHOOTER.LEFT_SHOOTER_MOTOR, true);
        RightShooterMotor = new KrakenX60Motor(SHOOTER.RIGHT_SHOOTER_MOTOR, false);
        MotorPID = new PIDProfile();
        MotorPID.setPID(SHOOTER.MOTOR_P, SHOOTER.MOTOR_I, SHOOTER.MOTOR_D);
        LeftShooterMotor.setFollowerTo(RightShooterMotor);
        LeftShooterMotor.withGains(MotorPID);
        LeftShooterMotor.configureMotionMagic(SHOOTER.MAX_ACCELERATION, SHOOTER.CRUISE_VELOCITY);
    }

    public void runAtSpeed(double desiredRPM){
        RightShooterMotor.setVelocity(desiredRPM);
    }

    public Command runAtSpeedCommand(double desiredRPM){
        return this.run(() -> runAtSpeed(desiredRPM));
    }

    public void stopShooter(){
        RightShooterMotor.setPercentOutput(0);
    }

    public Command stopShooterCommand(){
        return this.run(() -> stopShooter());
    }
        
}
