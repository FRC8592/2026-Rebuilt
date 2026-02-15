
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.autonomous.AutoManager;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Telemetry;
//import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.OdometryUpdates;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.vision.Vision;

import frc.robot.subsystems.Scoring;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import frc.robot.subsystems.Turret;
//import frc.robot.subsystems.Feeder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
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

    private final Telemetry logger = new Telemetry(SWERVE.MAX_SPEED);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

//     // robot subsystems
        private final Swerve swerve;
        private final OdometryUpdates odometryUpdates;
        private final Vision vision;
        // public final Indexer indexer;
        // public final Intake intake;
//     private final Launcher launcher;x
    private final Scoring scoring;
        //public final Shooter shooter;
        public final Turret turret;
        //public final Feeder feeder;

    // robot button triggers
    private final Trigger TESTING_TURRET = driverController.rightBumper();
    private final Trigger TESTING_TURRET_BACK = driverController.leftBumper();
    //private final Trigger TESTING_SHOOTER = driverController.x();
    //private final Trigger STOP_SHOOTER = driverController.a();
    private final Trigger RESET_POS = driverController.back();
    //private final Trigger TESTING_FEEDER = driverController.b();
    private final Trigger RESET_HEADING = driverController.back();
    private final Trigger SLOW_MODE = driverController.leftBumper();

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        swerve = new Swerve(drivetrain);
        vision = new Vision(VISION.CAMERA_NAME, VISION.CAMERA_OFFSETS);
        odometryUpdates = new OdometryUpdates(vision, swerve);
        // indexer = new Indexer();
        // intake = new Intake();
        //shooter = new Shooter();
        turret = new Turret(swerve);
        scoring = new Scoring(swerve, turret);
        //feeder = new Feeder();

        configureBindings();
        configureDefaults();

        AutoManager.prepare();
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

        //turret.setDefaultCommand(turret.TurrettoPosCommand());

    }

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        SLOW_MODE.onTrue(Commands.runOnce(() -> swerve.setSlowMode(true)))
                .onFalse(Commands.runOnce(() ->swerve.setSlowMode(false)));

        RESET_HEADING.onTrue(swerve.runOnce(() -> swerve.resetHeading()));

        //TESTING_SHOOTER.whileTrue(new DeferredCommand(() -> shooter.runAtSpeedCommand(1000), Set.of(shooter))).onFalse(shooter.stopShooterCommand());
        TESTING_TURRET.whileTrue(scoring.autoTurretCommand()).onFalse(turret.stopTurretCommand());
        //TESTING_TURRET_BACK.onTrue(turret.TurrettoPosCommand(-1)).onFalse(turret.stopTurretCommand());
       // TESTING_SHOOTER.onTrue(shooter.runAtSpeedCommand()).onFalse(shooter.stopShooterCommand());
        // RESET_POS.onTrue(turret.resetPosCommand());
        //TESTING_FEEDER.onTrue(feeder.runAtSpeedCommand()).onFalse(feeder.stopCommand());
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
