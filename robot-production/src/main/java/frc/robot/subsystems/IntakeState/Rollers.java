package frc.robot.subsystems.IntakeState;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE;
import frc.robot.helpers.TalonFXControl;

public class Rollers extends SubsystemBase{
    private TalonFXControl leftRollerMotor;
    private TalonFXControl rightRollerMotor;
    private RollerState wantedState;
    private RollerState currentState;

    public Rollers(){
        leftRollerMotor = new TalonFXControl(INTAKE.INTAKE_MOTOR_LEFT_CAN_ID, false, "leftRollerMotor");
        rightRollerMotor = new TalonFXControl(INTAKE.INTAKE_ROLLER_RIGHT_CAN_ID, false, "rightRollerMotor");

        leftRollerMotor.setCurrentLimit(INTAKE.ROLLER_CURRENT_LIMIT);
        rightRollerMotor.setCurrentLimit(INTAKE.ROLLER_CURRENT_LIMIT);
        
        leftRollerMotor.setFollower(rightRollerMotor, true);

        wantedState = RollerState.STOP;
    }

    public enum RollerState{
        FORWARD,
        REVERSE,
        STOP
    }

    public void setWantedState(RollerState wantedState){
        this.wantedState = wantedState;
    }

    private RollerState handleStateTransition(){ // Making it easier to handle subsystems in which you cannot go from one to another easily
        
        return switch (wantedState) {
            case FORWARD -> RollerState.FORWARD;
            case REVERSE -> RollerState.REVERSE;
            case STOP -> RollerState.STOP;
        };

    }

    private void applyState(){
        switch (currentState) {
            case FORWARD:
                rightRollerMotor.setVoltage(11);
            case REVERSE:
                rightRollerMotor.setVoltage(-11);
            case STOP:
                rightRollerMotor.setVoltage(0);

        }
    }

    @Override
    public void periodic(){
        currentState = handleStateTransition();
        Logger.recordOutput("Rollers/WantedState", wantedState.toString());
        Logger.recordOutput("Rollers/CurrentState", currentState.toString());
        applyState();
    }
}
