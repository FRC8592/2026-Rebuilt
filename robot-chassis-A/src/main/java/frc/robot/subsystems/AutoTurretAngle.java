package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;
import java.lang.Math;


public class AutoTurretAngle extends SubsystemBase{
    private double hubLocationX = 4.02844;
    private double hubLocationY = 4.445;
    public double rawAngle = 0;

    public AutoTurretAngle(){
    }

    //  @Override
    //  public void periodic(){
    //      rawAngle = TurretAngleCalc(swerve.getCurrentOdometryPosition(), new Pose2d(hubLocationX, hubLocationY, swerve.getYaw()));
    // }

    public double TurretAngleCalc(Pose2d robotPosition, Pose2d targetLocation){
        double targetRelativeX = targetLocation.getX() - robotPosition.getX();
        double targetRelativeY = targetLocation.getY() - robotPosition.getY();

        double triangleAngle = Math.toDegrees(Math.atan(targetRelativeY/targetRelativeX));
        SmartDashboard.putNumber("Triangle Angle", triangleAngle);

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
        
        //robotAngle is in degrees
        double robotAngle = robotPosition.getRotation().getDegrees();

        //angle robot has to turn it if is at angle robotAngle
        double turretTurn = thetaR - robotAngle;
        SmartDashboard.putNumber("Turret Angle", turretTurn);

        return turretTurn;
    }
}
