package frc.robot.commands.autonomous.autos;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.commands.autonomous.AutoCommand;

public class ShootThree extends AutoCommand {
    public ShootThree(){
        super(
            launcher.setLauncherCommand(0.44, 0.3)
            .andThen(new WaitUntilCommand(1))
            .andThen(indexer.setMotorPercentOutputCommand(1))
        );
    }
}
