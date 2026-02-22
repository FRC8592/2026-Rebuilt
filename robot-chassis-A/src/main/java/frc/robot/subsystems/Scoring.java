package frc.robot.subsystems;

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

        turret = new Turret(swerve);
        shooter = new Shooter();
        intake = new Intake();
        indexer = new Indexer();
    }
    
}
