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

    // TODO: implement the operator controller
    // private static final CommandGenericHID operatorController = new
    // CommandXboxController(
    // CONTROLLERS.OPERATOR_PORT
    // );

    private final Telemetry logger = new Telemetry(SWERVE.MAX_SPEED);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // robot subsystems
    private final Swerve swerve;
    private final OdometryUpdates odometryUpdates;
    private final Vision vision;
    private final Indexer indexer;
    private final Intake intake;
    private final Launcher launcher;
    private final Scoring scoring;

    // robot button triggers
    private final Trigger RESET_HEADING = driverController.back();
    // private final Trigger SLOW_MODE = driverController.leftBumper();
    // TODO: map these to the operator controller
    private final Trigger LAUNCH_NORMAL = operatorController.b();
    private final Trigger LAUNCH_CLOSE = operatorController.x();
    private final Trigger LAUNCH_HIGH = operatorController.y();
    private final Trigger LAUNCH_LOW = operatorController.a();

    private final Trigger RUN_INDEXER = driverController.rightTrigger();
    // private final Trigger INTAKE_TO_INDEXER = driverController.leftBumper();

    private final Trigger INTAKE_DEPLOY = driverController.leftBumper();
    private final Trigger INTAKE_STOW = driverController.rightBumper();

    private final Trigger INTAKE = driverController.leftTrigger();
    private final Trigger OUTTAKE = driverController.povUp();
    private final Trigger REVERSE_INTAKE = driverController.povLeft();
    // private final

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        swerve = new Swerve(drivetrain);
        vision = new Vision(VISION.CAMERA_NAME, VISION.CAMERA_OFFSETS);
        odometryUpdates = new OdometryUpdates(vision, swerve);
        launcher = new Launcher();
        indexer = new Indexer();
        intake = new Intake();
        scoring = new Scoring(intake, indexer, launcher);

        configureBindings();
        configureDefaults();
        passSubsystems();

        AutoManager.prepare();
    }

    private void passSubsystems(){
        LargeCommand.addSubsystems(swerve);
        AutoCommand.addSubsystems(swerve, launcher, indexer, intake);
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults() {
        // Set the swerve's default command to drive with joysticks

        setDefaultCommand(swerve, swerve.run(() -> {
            swerve.drive(swerve.processJoystickInputs(
                    -driverController.getLeftX(),
                    -driverController.getLeftY(),
                    -driverController.getRightX()));
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

        indexer.setDefaultCommand(
                new RunCommand(() -> indexer.autoIndex(), indexer));

        launcher.setDefaultCommand(
                new RunCommand(() -> launcher.setLauncherPercentOutput(0.4, 0.4), launcher));

    }

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        // SLOW_MODE.onTrue(Commands.runOnce(() ->
        // swerve.setSlowMode(true)).ignoringDisable(true))
        // .onFalse(Commands.runOnce(() ->
        // swerve.setSlowMode(false)).ignoringDisable(true));

        RESET_HEADING.onTrue(swerve.runOnce(() -> swerve.resetHeading()));

        LAUNCH_NORMAL.whileTrue(new DeferredCommand(() -> launcher.setLauncherCommand(0.4, 0.4), Set.of(launcher)))
                .onFalse(launcher.stopLauncherCommand());

        LAUNCH_CLOSE.whileTrue(new DeferredCommand(() -> launcher.setLauncherCommand(0.44, 0.3), Set.of(launcher)))
                .onFalse(launcher.stopLauncherCommand());

        LAUNCH_LOW.whileTrue(new DeferredCommand(() -> launcher.setLauncherCommand(0.23, 0.23), Set.of(launcher)))
                .onFalse(launcher.stopLauncherCommand());

        LAUNCH_HIGH.whileTrue(new DeferredCommand(() -> launcher.setLauncherCommand(0.4, 0.4), Set.of(launcher)))
                .onFalse(launcher.stopLauncherCommand());

        RUN_INDEXER
                .whileTrue(new RunCommand(() -> indexer.run(3, 1)).alongWith(new RunCommand(() -> indexer.run(2, 1))))
                .onFalse(indexer.stopMotorCommand(3));

        // RUN_INDEXER
        //         .whileTrue(new RunCommand(() -> indexer.shoot(1)))
        //         .onFalse(new RunCommand(() -> indexer.stopShoot()));

        INTAKE_DEPLOY.whileTrue(intake.deployIntakeCommand()).onFalse(intake.stopIntakePivotCommand());

        INTAKE_STOW.whileTrue(intake.stowIntakeCommand()).onFalse(intake.stopIntakePivotCommand());

        INTAKE.whileTrue(intake.runIntakeCommand()).onFalse(intake.stopIntakeCommand());

        OUTTAKE.whileTrue(intake.reverseIntakeCommand().alongWith(indexer.setIndexerReverseCommand()))
                .onFalse(intake.stopIntakeCommand().alongWith(indexer.setIndexerNormalCommand()));

        REVERSE_INTAKE.whileTrue(indexer.setIndexerReverseCommand()).onFalse(indexer.setIndexerNormalCommand());

        // INTAKE_TO_INDEXER.whileTrue(new DeferredCommand(() -> scoring.intakeLunite(),
        // Set.of(scoring))
        // ).onFalse(scoring.stowIntake());
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return AutoManager.getAutonomousCommand();
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
