package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;

import frc.robot.Constants.*;

public class Scoring extends SubsystemBase{
    private Swerve swerve;
    private Turret turret;
    private Pose2d targetPosition;
    
    public Scoring(Swerve swerve){
        this.swerve = swerve;
        turret = new Turret(swerve);
    }

    // @Override
    // public void periodic(){
    //     targetPosition = new Pose2d(4.02844, 4.445, new Rotation2d(0));
    //     turret.TurrettoPos(targetPosition);
    // }

    public Command autoTurretCommand(){
        return this.runOnce(() -> turret.setToTargetCommand(new Pose2d(SCORING.HUB_X, SCORING.HUB_Y, swerve.getYaw())));
    }

}
