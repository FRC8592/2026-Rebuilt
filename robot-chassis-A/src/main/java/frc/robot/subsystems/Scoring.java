package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.subsystems.swerve.Swerve;

public class Scoring extends SubsystemBase{
    Swerve swerve;
    Turret turret;
    Shooter shooter;
    Indexer indexer;
    Intake intake;

    public Scoring(Swerve swerve){
        this.swerve = swerve;

        turret = new Turret();
        shooter = new Shooter();
        intake = new Intake();
        indexer = new Indexer();
    }

    @Override
    public void periodic(){
        //will currently only track the red hub
        turret.TurrettoAngle(swerve.getCurrentOdometryPosition(), new Pose2d(4.02844, 4.445, swerve.getYaw()));
    }
    
}
