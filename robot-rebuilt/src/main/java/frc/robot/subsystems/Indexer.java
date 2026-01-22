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


public class Indexer extends SubsystemBase{
    private SparkFlexMotor IndexerMotor1;
    private SparkFlexMotor IndexerMotor2;

    private PIDProfile MotorPID;
    private double POld1 = INDEXER.INDEXER1_P;
    private double IOld1 = INDEXER.INDEXER1_I;
    private double DOld1 = INDEXER.INDEXER1_D;

    private double POld2 = INDEXER.INDEXER2_P;
    private double IOld2 = INDEXER.INDEXER2_I;
    private double DOld2 = INDEXER.INDEXER2_D;

    public Indexer(){
        IndexerMotor1 = new SparkFlexMotor(INDEXER.INDEXER1_CAN_ID, true);
        IndexerMotor2 = new SparkFlexMotor(INDEXER.INDEXER2_CAN_ID, true);

        MotorPID = new PIDProfile();
        MotorPID.setSlot(0);
        MotorPID.setPID(INDEXER.INDEXER1_P, INDEXER.INDEXER1_I,INDEXER.INDEXER1_D);
        MotorPID.setPID(INDEXER.INDEXER2_P, INDEXER.INDEXER2_I,INDEXER.INDEXER2_D);
        IndexerMotor1.withGains(MotorPID);
        IndexerMotor1.setCurrentLimit(80);
        IndexerMotor2.withGains(MotorPID);
        IndexerMotor2.setCurrentLimit(80);
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
        IndexerMotor1.setVelocity(RPM);
        IndexerMotor2.setVelocity(RPM);
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
        if((P != POld1) || (I!= IOld1) || (D!= DOld1)){
            System.out.println("Going into if statement in updatePID method");
            MotorPID.setPID(P, I, D);
            IndexerMotor1.withGains(MotorPID);
            POld1 = P;
            IOld1 = I;
            DOld1 = D;
        }

        if((P != POld2) || (I!= IOld2) || (D!= DOld2)){
            System.out.println("Going into if statement in updatePID method");
            MotorPID.setPID(P, I, D);
            IndexerMotor1.withGains(MotorPID);
            POld2 = P;
            IOld2 = I;
            DOld2 = D;
        }      
    }
    public void stopShooter(){
        IndexerMotor1.setPercentOutput(0);
        IndexerMotor2.setPercentOutput(0);
    }

    public Command stopShooterCommand(){
        return this.runOnce(() -> stopShooter());
    }

    public double getVelocity1(){
        return IndexerMotor1.getVelocityRPM();
    }

    public double getVelocity2(){
        return IndexerMotor2.getVelocityRPM();
    }
    //This is simply for calculation to get the ball landing in the center of the goal, based on the distance to the hub
    //This is very much a theoretical implementation, simply putting in just the math
    // public double DistanceToRPM(double theta, double dis){
    //     return (1/Math.cos(theta))* (Math.sqrt((0.5 * 9.81 * Math.pow(dis,2))/(SHOOTER.SHOOTER_HEIGHT + Math.tan(theta) - SHOOTER.HUB_HEIGHT)));
    // }

    @Override
    public void periodic(){
        // updatePID();
        Logger.recordOutput("Motor Velocity(1) RPM", getVelocity1());
        Logger.recordOutput("Motor Velocity(2) RPM", getVelocity2());
    }
        
}