package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.swerve.Swerve;



public class Scoring extends SubsystemBase{
    Swerve swerve;
    Turret turret;
    public Scoring(Swerve swerve, Turret turret){
        this.swerve = swerve;
        this.turret = turret;
    }

    public Command autoTurretCommand(){
        return turret.TurrettoPosCommand(new Pose2d(4.02844, 4.445, swerve.getYaw()));
    }
    
}
