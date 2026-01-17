// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.OpenOption;
import java.util.Set;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.*;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.autonomous.AutoManager;
import frc.robot.commands.largecommands.LargeCommand;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
    private static final CommandXboxController driverController = new CommandXboxController(
            CONTROLLERS.DRIVER_PORT);

    private static final CommandXboxController operatorController = new CommandXboxController(
            CONTROLLERS.OPERATOR_PORT);

    // robot subsystems
    private final Indexer_2 indexer;
    // robot button triggers
    private final Trigger RUN_INDEXER = driverController.a();

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        indexer = new Indexer_2();

        configureBindings();
    }

    /**
     * Configure all button bindings
     */
    private void configureBindings() {

        RUN_INDEXER.whileTrue(new DeferredCommand(() ->
                indexer.setIndexerCommand(SmartDashboard.getNumber("spinnerMotor", 0.2)), 
                Set.of(indexer))).onFalse(indexer.stopCommand());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return new InstantCommand();
    }

    /**
     * Set the default command of a subsystem (what to run if no other command
     * requiring it is running).
     * <p>
     * NOTE: all subsystems also have a setDefaultCommand method; this version
     * includes a check for
     * default commands that cancel incoming commands that require the subsystem.
     * Unless you're sure
     * of what you're doing, you should use this one.
     *
     * @param subsystem the subsystem to apply the default command to
     * @param command   to command to set as default
     */
    private void setDefaultCommand(SubsystemBase subsystem, Command command) {
        if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
            subsystem.setDefaultCommand(command);
        } else {
            // If you want to force-allow setting a cancel-incoming default command,
            // directly call `subsystem.setDefaultCommand()` instead
            throw new UnsupportedOperationException("Can't set a default command that cancels incoming!");
        }
    }

}
