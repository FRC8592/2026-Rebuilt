package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import java.lang.Math;

import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.*;
import frc.robot.helpers.PIDProfile;
import frc.robot.helpers.motor.NewtonMotor;
import frc.robot.helpers.motor.talonfx.KrakenX60Motor;
import frc.robot.helpers.motor.spark.SparkFlexMotor;


public class Intake extends SubsystemBase{
    private SparkFlexMotor IntakeMotor; 
    private PIDProfile MotorPID;
    private double POld = INTAKE.INTAKE_P;
    private double IOld = INTAKE.INTAKE_I;
    private double DOld = INTAKE.INTAKE_D;

    public Intake(){
        IntakeMotor = new SparkFlexMotor(INTAKE.INTAKE_MOTOR_CAN_ID, true);
        MotorPID = new PIDProfile();
        MotorPID.setSlot(0);
        MotorPID.setPID(INTAKE.INTAKE_P, INTAKE.INTAKE_I, INTAKE.INTAKE_D);
        IntakeMotor.withGains(MotorPID);
        IntakeMotor.setCurrentLimit(80);
        //RightShooterMotor.setFollowerTo(LeftShooterMotor, true);
        SmartDashboard.putNumber("P", 0.01);
        SmartDashboard.putNumber("I", 0.0);
        SmartDashboard.putNumber("D", 0.0);
        SmartDashboard.putNumber("Vi", 3000);
        //LeftShooterMotor.configureMotionMagic(SHOOTER.MAX_ACCELERATION, SHOOTER.CRUISE_VELOCITY);
    }

    public void runAtSpeed(double desiredRPM){
        double RPM = SmartDashboard.getNumber("Vi", 0);
        System.out.println("Shooter method is running");
        System.out.println("RPM Set " + RPM);
        IntakeMotor.setVelocity(RPM);
    }

    public Command runAtSpeedCommand(){
        double RPM = SmartDashboard.getNumber("Vi", 0);
        return this.runOnce(() -> runAtSpeed(RPM));
    }
    public void updatePID(){
        //System.out.println("Going into updatePID Method");
        double P = SmartDashboard.getNumber("P", 0.001);
        double I = SmartDashboard.getNumber("I", 0.0);
        double D = SmartDashboard.getNumber("D", 0.0);
        //System.out.println("P Value from SmartDashboard "+ P + " P Old Value " + POld);
        if((P != POld) || (I!= IOld) || (D!= DOld)){
            System.out.println("Going into if statement in updatePID method");
            MotorPID.setPID(P, I, D);
            IntakeMotor.withGains(MotorPID);
            POld = P;
            IOld = I;
            DOld = D;
        }
    }
    public void stopShooter(){
        IntakeMotor.setPercentOutput(0);
    }

    public Command stopShooterCommand(){
        return this.runOnce(() -> stopShooter());
    }

    public double getVelocity(){
        return IntakeMotor.getVelocityRPM();
    }
    //This is simply for calculation to get the ball landing in the center of the goal, based on the distance to the hub
    //This is very much a theoretical implementation, simply putting in just the math
    // public double DistanceToRPM(double theta, double dis){
    //     return (1/Math.cos(theta))* (Math.sqrt((0.5 * 9.81 * Math.pow(dis,2))/(SHOOTER.SHOOTER_HEIGHT + Math.tan(theta) - SHOOTER.HUB_HEIGHT)));
    // }

    @Override
    public void periodic(){
        // updatePID();
        Logger.recordOutput("Motor Velocity RPM", getVelocity());
    }
        
}