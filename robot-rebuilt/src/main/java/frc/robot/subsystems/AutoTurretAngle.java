package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.lang.Math;

import frc.robot.Constants.*;
import frc.robot.subsystems.swerve.Swerve;

public class AutoTurretAngle extends SubsystemBase{
    private double hubLocationX = 4.02844;
    private double hubLocationY = 4.445;
    public double rawAngle = 0;

    Swerve swerve;

    public AutoTurretAngle(Swerve swerve){
        this.swerve = swerve;
    }

    @Override
    public void periodic(){
        rawAngle = TurretAngleCalc(swerve.getCurrentOdometryPosition());

    }

    public double TurretAngleCalc(Pose2d robotPosition){
        System.out.println("Swerve Position Fed X, Y" + robotPosition.getX() + ", " + robotPosition.getY());
        double hubRelativeX = hubLocationX - robotPosition.getX();
        double hubRelativeY = hubLocationY - robotPosition.getY();

        double triangleAngle = Math.toDegrees(Math.atan(hubRelativeY/hubRelativeX));
        SmartDashboard.putNumber("Triangle Angle", triangleAngle);

        //angle robot has to turn if it is at angle 0
        double thetaR = 0;

        if (hubRelativeX > 0 && hubRelativeY >= 0){
            thetaR = triangleAngle;
        }
        else if (hubRelativeX < 0 && hubRelativeY >= 0){
            thetaR = 180 + triangleAngle;
        }
        else if (hubRelativeX > 0 && hubRelativeY <= 0){
            thetaR = triangleAngle;
        }
        else if (hubRelativeX < 0 && hubRelativeY <= 0){
            thetaR = -180 + triangleAngle;
        }
        else if (hubRelativeX == 0 && hubRelativeY > 0){
            thetaR = 270;
        }
        else if (hubRelativeX == 0 && hubRelativeY < 0){
            thetaR = 90;
        }
        else{
            System.out.println("you are on top of the hub");
        }
        
        //robotAngle is in degrees
        double robotAngle = robotPosition.getRotation().getDegrees();

        //angle robot has to turn it if is at angle robotAngle
        double turretTurn = thetaR - robotAngle;
        SmartDashboard.putNumber("Turret Angle", turretTurn);
        return turretTurn;
    }
}
