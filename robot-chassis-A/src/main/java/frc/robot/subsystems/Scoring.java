package frc.robot.subsystems;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.Swerve;

public class Scoring extends SubsystemBase{
    Swerve swerve;
    public Turret turret;
    public Shooter shooter;
    public Indexer indexer;
    public Intake intake;

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
        //turret.TurrettoAngle(swerve.getCurrentOdometryPosition(), new Pose2d(4.02844, 4.445, swerve.getYaw()));
    }

    public Command scoring(){
        return new DeferredCommand(() -> shooter.runAtSpeedCommand(), Set.of(this.shooter))
        .alongWith(new WaitCommand(1).andThen(new DeferredCommand(() -> indexer.runIndexerCommand() , Set.of(this.indexer))));
    }


    
}
