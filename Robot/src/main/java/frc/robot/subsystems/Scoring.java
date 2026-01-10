package frc.robot.subsystems;

import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
//import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE;
import frc.robot.Constants.SCORING;

public class Scoring extends SubsystemBase{
    private Intake intake;
    private Indexer indexer;
    private Launcher launcher;
    private double intakePos;


    public Scoring(Intake intake, Indexer indexer, Launcher launcher){
        this.intake = intake;
        this.indexer = indexer;
        this.launcher = launcher;
    }

    public Command deployIntake() {
        return this.runOnce(() -> intake.deployIntakeCommand());
    }

    public Command stowIntake(){
        return this.runOnce(() -> intake.stowIntakeCommand());
    }

    public Command stopIntakePivot() {
        return this.runOnce(() -> intake.stopIntakePivotCommand());
    }

    public Command runIntake() {
        return this.runOnce(() -> intake.runIntakeCommand());
    }
    
    public Command stopIntake() {
        return this.runOnce(() -> intake.stopIntakeCommand());
    }
 
    public void periodic(){
        Logger.recordOutput(SCORING.LOG_PATH+ "Intake Position", intakePos);
    }
}