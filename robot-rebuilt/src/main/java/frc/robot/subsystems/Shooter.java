package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;
import frc.robot.helpers.motor.spark.SparkFlexMotor;


public class Shooter extends SubsystemBase{
    private SparkFlexMotor LeftShooterMotor;
    private SparkFlexMotor RightShooterMotor;
    private PIDProfile MotorPID;
    private double POld = SHOOTER.MOTOR_P;
    private double IOld = SHOOTER.MOTOR_I;
    private double DOld = SHOOTER.MOTOR_D;

    public Shooter(){
        LeftShooterMotor = new SparkFlexMotor(SHOOTER.LEFT_SHOOTER_MOTOR, true);
        RightShooterMotor = new SparkFlexMotor(SHOOTER.RIGHT_SHOOTER_MOTOR, false);
        //LeftShooterMotor.setIdleMode(IdleMode.kCoast);
        MotorPID = new PIDProfile();
        MotorPID.setSlot(0);
        MotorPID.setPID(SHOOTER.MOTOR_P, SHOOTER.MOTOR_I, SHOOTER.MOTOR_D);
        //LeftShooterMotor.setFollowerTo(RightShooterMotor);
        LeftShooterMotor.withGains(MotorPID);
        SmartDashboard.putNumber("P", 0.001);
        SmartDashboard.putNumber("I", 0.0);
        SmartDashboard.putNumber("D", 0.0);
        //LeftShooterMotor.configureMotionMagic(SHOOTER.MAX_ACCELERATION, SHOOTER.CRUISE_VELOCITY);
    }

    public void runAtSpeed(double desiredRPM){
        System.out.println("Shooter method is running");
        System.out.println("RPM Set " + desiredRPM);
        LeftShooterMotor.setVelocity(desiredRPM);
    }

    public Command runAtSpeedCommand(double desiredRPM){
        return this.runOnce(() -> runAtSpeed(desiredRPM));
    }
    public void updatePID(){
        double P = SmartDashboard.getNumber("P", 0.001);
        double I = SmartDashboard.getNumber("I", 0.0);
        double D = SmartDashboard.getNumber("D", 0.0);
        if(P != POld || I!= IOld || D!= DOld){
            MotorPID.setPID(P, I, D);
            LeftShooterMotor.withGains(MotorPID);
            POld = P;
            IOld = I;
            DOld = D;
        }
    }
    public void stopShooter(){
        //LeftShooterMotor.setVelocity(0);
        LeftShooterMotor.setPercentOutput(0);
    }

    public Command stopShooterCommand(){
        return this.run(() -> stopShooter());
    }

    public double getVelocity(){
        return LeftShooterMotor.getVelocityRPM();
    }

    @Override
    public void periodic(){
        Logger.recordOutput("Motor Velocity RPM", getVelocity());
    }
        
}
