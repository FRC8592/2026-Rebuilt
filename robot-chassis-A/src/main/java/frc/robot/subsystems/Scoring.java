package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SCORING;
import frc.robot.subsystems.swerve.Swerve;



public class Scoring extends SubsystemBase{
    Swerve swerve;
    Turret turret;
    Shooter shooter;

    public Scoring(Swerve swerve, Turret turret, Shooter shooter){
        this.swerve = swerve;
        this.turret = turret;
        this.shooter = shooter;
    }

    public Command autoTurretCommand(){
        return turret.TurrettoPosCommand(new Pose2d(SCORING.HUB_X, SCORING.HUB_Y, swerve.getYaw()));
    }
    
}
