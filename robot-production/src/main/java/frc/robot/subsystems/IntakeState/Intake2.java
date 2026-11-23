package frc.robot.subsystems.IntakeState;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.IntakeState.Extension.ExtensionState;
import frc.robot.subsystems.IntakeState.Rollers.RollerState;

public class Intake2 extends SubsystemBase{
    private Extension extension;
    private Rollers rollers;

    private IntakeState wantedState;
    private IntakeState currentState;

    public Intake2(){
        extension = new Extension();
        rollers = new Rollers();

        wantedState = IntakeState.IDLE_IN;

    }

    public enum IntakeState{
        IDLE_OUT,
        IDLE_IN,
        OUT_INTAKE,
        IN_INTAKE, // Used to agitate balls sometimes
        OUT_OUTTAKE,
        STOP

    }

    public void setWantedState(IntakeState wantedState){
        this.wantedState = wantedState;
    }

    private IntakeState handleStateTransition(){ // Making it easier to handle subsystems in which you cannot go from one to another easily
        
        return switch (wantedState) {
            case IDLE_OUT -> IntakeState.IDLE_OUT;
            case IDLE_IN -> IntakeState.IDLE_IN;
            case OUT_INTAKE -> IntakeState.OUT_INTAKE;
            case IN_INTAKE -> IntakeState.IN_INTAKE;
            case OUT_OUTTAKE -> IntakeState.OUT_OUTTAKE;
            case STOP -> IntakeState.STOP;
        };

    }

    private void applyState(){
        switch (currentState) {
            case IDLE_OUT:
                extension.setWantedState(ExtensionState.OUT);
                rollers.setWantedState(RollerState.STOP);
            case IDLE_IN:
                extension.setWantedState(ExtensionState.IN);
                rollers.setWantedState(RollerState.STOP);
            case OUT_INTAKE:
                extension.setWantedState(ExtensionState.OUT);
                rollers.setWantedState(RollerState.FORWARD);
            case IN_INTAKE:
                extension.setWantedState(ExtensionState.IN);
                rollers.setWantedState(RollerState.FORWARD);
            case OUT_OUTTAKE:
                extension.setWantedState(ExtensionState.OUT);
                rollers.setWantedState(RollerState.REVERSE);
            case STOP:
                extension.setWantedState(ExtensionState.STOP);
                rollers.setWantedState(RollerState.STOP);


        }
    }

    @Override
    public void periodic(){
        currentState = handleStateTransition();
        Logger.recordOutput("Intake/WantedState", wantedState.toString());
        Logger.recordOutput("Intake/CurrentState", currentState.toString());
        applyState();
    }

    public boolean isExtensionIn(){
        return extension.isIn();
    }

    public boolean isExtensionOut(){
        return extension.isOut();
    }
}


