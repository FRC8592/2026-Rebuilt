package frc.robot.commands.autonomous;

import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Scoring;

public final class AutoCommands {
    public static void registerNamedCommands(Scoring scoring) {
        NamedCommands.registerCommand("Shoot", scoring.indexer.runIndexerCommand());
        NamedCommands.registerCommand("StopShoot", scoring.indexer.stopCommand());
        NamedCommands.registerCommand("ToggleHubTracking", scoring.toggleTrackingCommand());

        NamedCommands.registerCommand("SqueezeShoot", Commands.defer(() -> {
            Command squeezeRoutine = Commands.sequence(
                    scoring.intake.retractWithRollersCommand(),
                    Commands.waitSeconds(1.5),
                    scoring.intake.stopRollerCommand(),
                    scoring.intake.stopExtendCommand());

            return Commands.sequence(
                    scoring.indexer.runIndexerCommand(),
                    Commands.runOnce(() -> CommandScheduler.getInstance().schedule(squeezeRoutine)));
        }, Set.of()));
        
    }


    public static void registerEventTriggers(Scoring scoring) {
        new EventTrigger("RunIntake").onTrue(scoring.intake.runIntakeRollersCommand());
        new EventTrigger("DeployIntake").onTrue(scoring.intake.extendIntakeCommand().withTimeout(0.05));
        new EventTrigger("StopIntake")
                .onTrue(scoring.intake.stopRollerCommand().andThen(scoring.intake.stopExtendCommand()));
    }

    public static void registerAll(Scoring scoring) {
        registerNamedCommands(scoring);
        registerEventTriggers(scoring);
    }

    private AutoCommands() {
    }
}