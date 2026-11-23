package frc.robot.subsystems.IntakeState;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.INTAKE;
import frc.robot.helpers.SparkFlexControl;
import frc.robot.subsystems.IntakeState.Intake2.IntakeState;

public class Extension extends SubsystemBase{

    SparkFlexControl extendMotor;
    private ExtensionState wantedState;
    private ExtensionState currentState;
    
    public Extension(){
        extendMotor = new SparkFlexControl(INTAKE.INTAKE_EXTEND_CAN_ID, true, "intakeExtendMotor");

        extendMotor.setCurrentLimit(INTAKE.EXTEND_CURRENT_LIMIT);

        extendMotor.setPIDF(INTAKE.INTAKE_EXTEND_P, INTAKE.INTAKE_EXTEND_I, INTAKE.INTAKE_EXTEND_D);
        extendMotor.smartMotion(INTAKE.CRUISE_VELOCITY,INTAKE.MAX_ACCELERATION,10);

        extendMotor.setReverseSoftLimit(INTAKE.EXTEND_SOFT_LIMIT);
        extendMotor.setForwardSoftLimit(INTAKE.EXTEND_ROTATIONS); // Constant named weird, basically max the motor can go out in the forward direction

        wantedState = ExtensionState.IN;
    }

    public enum ExtensionState{
        IN,
        OUT,
        STOP
    }

    public void setWantedState(ExtensionState wantedState){
        this.wantedState = wantedState;
    }

    private ExtensionState handleStateTransition(){ // Making it easier to handle subsystems in which you cannot go from one to another easily
        
        return switch (wantedState) {
            case IN -> ExtensionState.IN;
            case OUT -> ExtensionState.OUT;
            case STOP -> ExtensionState.STOP;
        };

    }

    private void applyState(){
        switch (currentState) {
            case IN:
                extendMotor.setPosition(INTAKE.EXTEND_SOFT_LIMIT);
            case OUT:
                extendMotor.setPosition(INTAKE.EXTEND_ROTATIONS);
            case STOP:
                extendMotor.setVoltage(0);

        }
    }

    @Override
    public void periodic(){
        currentState = handleStateTransition();
        Logger.recordOutput("Extension/WantedState", wantedState.toString());
        Logger.recordOutput("Extension/CurrentState", currentState.toString());
        applyState();
    }

    public boolean isIn(){
        return extendMotor.getPosition() <= INTAKE.EXTEND_SOFT_LIMIT + 0.25;
    }

    public boolean isOut(){
        return extendMotor.getPosition() >= INTAKE.EXTEND_ROTATIONS - 0.25;
    }
}
