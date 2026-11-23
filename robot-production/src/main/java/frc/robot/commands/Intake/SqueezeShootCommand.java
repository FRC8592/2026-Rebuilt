package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeState.Intake2.IntakeState;
import frc.robot.subsystems.IntakeState.Intake2;

public class SqueezeShootCommand extends Command{
    private final Intake2 intake;
    private boolean retracting;

    public SqueezeShootCommand(Intake2 intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        retracting = true;
        intake.setWantedState(IntakeState.IN_INTAKE);
    }

    @Override
    public void execute() {
        if(retracting && intake.isExtensionIn()){
            retracting = false;
            intake.setWantedState(IntakeState.OUT_INTAKE);
        } else if(!retracting && intake.isExtensionOut()){
            retracting = true;
            intake.setWantedState(IntakeState.IN_INTAKE);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setWantedState(IntakeState.IDLE_IN);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
