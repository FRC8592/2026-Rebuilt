package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TURRET;

import org.littletonrobotics.junction.Logger;
import java.lang.Math;


public class AutoTurretAngle extends SubsystemBase{
    public double rawAngle = 0;

    // TODO: Delete empty constructor
    public AutoTurretAngle(){
    }


    /**
     * Compute the angle for the turret based on the position of target and the position and rotation of the robot.
     * @param robotPosition
     * @param targetLocation
     * @return The angle the turret needs to turn to face the target
     */
    public double TurretAngleCalc(Pose2d robotPosition, Pose2d targetLocation){
        double targetRelativeX = targetLocation.getX() - robotPosition.getX();
        double targetRelativeY = targetLocation.getY() - robotPosition.getY();

        double triangleAngle = Math.toDegrees(Math.atan(targetRelativeY/targetRelativeX));

        //angle robot has to turn if it is at angle 0
        double thetaR = 0;

        if (targetRelativeX > 0 && targetRelativeY >= 0){
            thetaR = triangleAngle;
        }
        else if (targetRelativeX < 0 && targetRelativeY >= 0){
            thetaR = 180 + triangleAngle;
        }
        else if (targetRelativeX > 0 && targetRelativeY <= 0){
            thetaR = triangleAngle;
        }
        else if (targetRelativeX < 0 && targetRelativeY <= 0){
            thetaR = -180 + triangleAngle;
        }
        else if (targetRelativeX == 0 && targetRelativeY > 0){
            thetaR = 270;
        }
        else if (targetRelativeX == 0 && targetRelativeY < 0){
            thetaR = 90;
        }
        else{
            System.out.println("you are on top of the hub");
        }
        
        // robotAngle is in degrees
        double robotAngle = robotPosition.getRotation().getDegrees();

        // Computer the angle offset from robot zero
        double turretTurn = thetaR - robotAngle;

        // Compute the angle offset accounting for turret zero
        turretTurn = turretTurn - TURRET.TURRET_ANGLE_OFFSET;
    
        return turretTurn;
    }
}
