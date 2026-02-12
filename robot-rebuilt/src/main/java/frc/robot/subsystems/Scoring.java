package frc.robot.subsystems;

import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swerve.Swerve;

import frc.robot.Constants.*;

public class Scoring extends SubsystemBase{
    private Swerve swerve;
    private Turret turret;
    private Pose2d targetPosition;
    private Command runCommand;
    private boolean interrupted;
    
    public Scoring(Swerve swerve, Turret turret){
        this.swerve = swerve;
        this.turret = turret;
    }

    @Override
    public void periodic(){
    }

    public Command autoTurretCommand(){
        runCommand = new DeferredCommand(() -> turret.setToTargetCommand(new Pose2d(SCORING.HUB_X, SCORING.HUB_Y, swerve.getYaw())), Set.of(turret));
        return runCommand;
    }

}
