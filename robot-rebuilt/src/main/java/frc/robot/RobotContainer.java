
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.CONTROLLERS;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.autonomous.AutoManager;
import frc.robot.commands.largecommands.LargeCommand;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer; 
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.Feeder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

//     private final Telemetry logger = new Telemetry(SWERVE.MAX_SPEED);
//     public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

//     // robot subsystems
//     private final Swerve swerve;
//     private final OdometryUpdates odometryUpdates;
//     private final Vision vision;
        // public final Indexer indexer;
        // public final Intake intake;
//     private final Launcher launcher;x
//     private final Scoring scoring;
        //public final Shooter shooter;
        public final Turret turret;
        public final Feeder feeder;

//     // robot button triggers
        private final Trigger TESTING_TURRET = driverController.rightBumper();
        private final Trigger TESTING_TURRET_BACK = driverController.leftBumper();
        //private final Trigger TESTING_SHOOTER = driverController.x();
        //private final Trigger STOP_SHOOTER = driverController.a();
        private final Trigger RESET_POS = driverController.back();
        //private final Trigger TESTING_FEEDER = driverController.b();
//     private final Trigger RESET_HEADING = driverController.back();
//     // private final Trigger SLOW_MODE = driverController.leftBumper();
//     // TODO: map these to the operator controller
//     private final Trigger LAUNCH_NORMAL = operatorController.b();
//     private final Trigger LAUNCH_CLOSE = operatorController.x();
//     private final Trigger LAUNCH_HIGH = operatorController.y();
//     private final Trigger LAUNCH_LOW = operatorController.a();

//     private final Trigger RUN_INDEXER = driverController.rightTrigger();
//     // private final Trigger INTAKE_TO_INDEXER = driverController.leftBumper();

//     private final Trigger INTAKE_DEPLOY = driverController.leftBumper();
//     private final Trigger INTAKE_STOW = driverController.rightBumper();

//     private final Trigger INTAKE = driverController.leftTrigger();
//     private final Trigger OUTTAKE = driverController.povUp();
//     private final Trigger REVERSE_INTAKE = driverController.povLeft();
    // private final

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        // swerve = new Swerve(drivetrain);
        // vision = new Vision(VISION.CAMERA_NAME, VISION.CAMERA_OFFSETS);
        // odometryUpdates = new OdometryUpdates(vision, swerve);
        // launcher = new Launcher();
        // indexer = new Indexer();
        // intake = new Intake();
        // scoring = new Scoring(intake, indexer, launcher);
        //shooter = new Shooter();
        turret = new Turret();
        feeder = new Feeder();

        configureBindings();
//         configureDefaults();
//         passSubsystems();

//         AutoManager.prepare();
//     }

//     private void passSubsystems(){
//         LargeCommand.addSubsystems(swerve);
//         AutoCommand.addSubsystems(swerve, launcher, indexer, intake);
     }

    /**
     * Configure default commands for the subsystems
     */
//     private void configureDefaults() {
//         // Set the swerve's default command to drive with joysticks

//         setDefaultCommand(swerve, swerve.run(() -> {
//             swerve.drive(swerve.processJoystickInputs(
//                     -driverController.getLeftX(),
//                     -driverController.getLeftY(),
//                     -driverController.getRightX()));
//         }).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

//         indexer.setDefaultCommand(
//                 new RunCommand(() -> indexer.autoIndex(), indexer));

//         launcher.setDefaultCommand(
//                 new RunCommand(() -> launcher.setLauncherPercentOutput(0.4, 0.4), launcher));

//     }

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        // SLOW_MODE.onTrue(Commands.runOnce(() ->
        // swerve.setSlowMode(true)).ignoringDisable(true))
        // .onFalse(Commands.runOnce(() ->
        // swerve.setSlowMode(false)).ignoringDisable(true));

        // RESET_HEADING.onTrue(swerve.runOnce(() -> swerve.resetHeading()));

        //TESTING_SHOOTER.whileTrue(new DeferredCommand(() -> shooter.runAtSpeedCommand(1000), Set.of(shooter))).onFalse(shooter.stopShooterCommand());
        TESTING_TURRET.onTrue(turret.TurrettoPosCommand(1)).onFalse(turret.stopTurretCommand());
        TESTING_TURRET_BACK.onTrue(turret.TurrettoPosCommand(-1)).onFalse(turret.stopTurretCommand());
       // TESTING_SHOOTER.onTrue(shooter.runAtSpeedCommand()).onFalse(shooter.stopShooterCommand());
        RESET_POS.onTrue(turret.resetPosCommand());
        //TESTING_FEEDER.onTrue(feeder.runAtSpeedCommand()).onFalse(feeder.stopCommand());

        // LAUNCH_NORMAL.whileTrue(new DeferredCommand(() -> launcher.setLauncherCommand(0.4, 0.4), Set.of(launcher)))
        //         .onFalse(launcher.stopLauncherCommand());

        // LAUNCH_CLOSE.whileTrue(new DeferredCommand(() -> launcher.setLauncherCommand(0.44, 0.3), Set.of(launcher)))
        //         .onFalse(launcher.stopLauncherCommand());

        // LAUNCH_LOW.whileTrue(new DeferredCommand(() -> launcher.setLauncherCommand(0.23, 0.23), Set.of(launcher)))
        //         .onFalse(launcher.stopLauncherCommand());

        // LAUNCH_HIGH.whileTrue(new DeferredCommand(() -> launcher.setLauncherCommand(0.4, 0.4), Set.of(launcher)))
        //         .onFalse(launcher.stopLauncherCommand());

        // RUN_INDEXER
        //         .whileTrue(new RunCommand(() -> indexer.run(3, 1)).alongWith(new RunCommand(() -> indexer.run(2, 1))))
        //         .onFalse(indexer.stopMotorCommand(3));

        // // RUN_INDEXER
        // //         .whileTrue(new RunCommand(() -> indexer.shoot(1)))
        // //         .onFalse(new RunCommand(() -> indexer.stopShoot()));

        // INTAKE_DEPLOY.whileTrue(intake.deployIntakeCommand()).onFalse(intake.stopIntakePivotCommand());

        // INTAKE_STOW.whileTrue(intake.stowIntakeCommand()).onFalse(intake.stopIntakePivotCommand());

        // INTAKE.whileTrue(intake.runIntakeCommand()).onFalse(intake.stopIntakeCommand());

        // OUTTAKE.whileTrue(intake.reverseIntakeCommand().alongWith(indexer.setIndexerReverseCommand()))
        //         .onFalse(intake.stopIntakeCommand().alongWith(indexer.setIndexerNormalCommand()));

        // REVERSE_INTAKE.whileTrue(indexer.setIndexerReverseCommand()).onFalse(indexer.setIndexerNormalCommand());

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

  private void setDefaultCommand(SubsystemBase subsystem, Command command) {
        if (command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf) {
            subsystem.setDefaultCommand(command);
        } else {
            // If you want to force-allow setting a cancel-incoming default command,
            // directly call subsystem.setDefaultCommand() instead
            throw new UnsupportedOperationException("Can't set a default command that cancels incoming!");
        }
    }
}
