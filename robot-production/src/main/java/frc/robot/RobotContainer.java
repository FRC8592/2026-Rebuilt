// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.EventMarker;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CONTROLLERS;
import frc.robot.Constants.VISION;
import frc.robot.commands.autonomous.AutoManager;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.OdometryUpdates;
import frc.robot.subsystems.Scoring;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.vision.Vision;

public class RobotContainer {
  private static final CommandXboxController driverController =
      new CommandXboxController(CONTROLLERS.DRIVER_PORT);
  private static final CommandXboxController operatorController =
      new CommandXboxController(CONTROLLERS.OPERATOR_PORT);

  // Robot subsystems
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Swerve swerve;
  private final Vision visionBack;
  private final Vision visionRight;
  private final Vision visionLeft;
  private final OdometryUpdates odometryUpdatesBack;
  private final OdometryUpdates odometryUpdatesRight;
  private final OdometryUpdates odometryUpdatesLeft;
  public final Scoring scoring;
  public final LEDs leds;

  // Driver Controls
  private final Trigger RESET_HEADING = driverController.back();

  private final Trigger ALIGN_HEADING = driverController.y();
  private final Trigger INTAKE_RUN = driverController.rightTrigger();
  private final Trigger INTAKE_REVERSE = driverController.rightBumper();
  private final Trigger INTAKE_EXTEND = driverController.leftBumper();
  private final Trigger INTAKE_RETRACT = driverController.leftTrigger();
  private final Trigger RESET_EXTEND = driverController.b();
  private final Trigger LOCK_WHEELS = driverController.x();

  private final Trigger SHOOT_SQUEEZE = driverController.a();

  // private final Trigger SNAP_TO = driverController.povUp();

  // Operator Controls
  private final Trigger ENABLE_TRACKING = operatorController.leftTrigger();
  private final Trigger SHOOT = operatorController.rightTrigger();

  private final Trigger REVERSE_TURRET_TESTING = operatorController.leftBumper();
  private final Trigger POSITIVE_TURRET_TESTING = operatorController.rightBumper();

  private final Trigger RESET_TURRET = operatorController.a();
  private final Trigger MANUAL_OVERRIDE = operatorController.back();
  // private final Trigger TURRET_TEST = operatorController.x();
  // private final Trigger TURRET_TEST_BACK = operatorController.a();

  // Controls for running sysId tests
  // private final Trigger QUASI_FORWARD = driverController.a();
  // private final Trigger QUASI_REVERSE = driverController.y();
  // private final Trigger DYNAMIC_FORWARD = driverController.b();
  // private final Trigger DYNAMIC_REVERSE = driverController.x();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    leds = new LEDs();
    swerve = new Swerve(drivetrain);
    scoring = new Scoring(swerve, leds);
    visionBack = new Vision(VISION.CAMERA_NAME_BACK, VISION.CAMERA_OFFSETS_BACK);
    visionRight = new Vision(VISION.CAMERA_NAME_RIGHT, VISION.CAMERA_OFFSETS_RIGHT);
    visionLeft = new Vision(VISION.CAMERA_NAME_LEFT, VISION.CAMERA_OFFSETS_LEFT);

    odometryUpdatesBack = new OdometryUpdates(visionBack, swerve);
    odometryUpdatesLeft = new OdometryUpdates(visionLeft, swerve);
    odometryUpdatesRight = new OdometryUpdates(visionRight, swerve);

    // TODO: Figure out the issues with these, they are very temporary
    NamedCommands.registerCommand("Shoot", scoring.indexer.runIndexerCommand());
    NamedCommands.registerCommand("StopShoot", scoring.indexer.stopCommand());
    new EventTrigger("RunIntake").whileTrue(scoring.intake.runIntakeRollersCommand());
    new EventTrigger("DeployIntake").whileTrue(scoring.intake.extendIntakeCommand());
    new EventTrigger("StopIntake")
        .onTrue(scoring.intake.stopRollerCommand().andThen(scoring.intake.stopExtendCommand()));
    // new
    // EventTrigger("RetractIntake").whileTrue(scoring.intake.retractIntakeCommand(6));
    new EventTrigger("ToggleHubTracking").onTrue(scoring.toggleTrackingCommand());
    new EventTrigger("TurnOffTracking").onTrue(scoring.toggleTrackingCommand());
    new EventTrigger("StopShoot").onTrue(scoring.indexer.stopCommand());
    new EventTrigger("Wait").onTrue(Commands.waitSeconds(4.0));
    new EventTrigger("WaitAndShoot")
        .onTrue(Commands.waitSeconds(2).andThen(scoring.indexer.runIndexerCommand()));

    new EventTrigger("ShootWhileSqueezing").onTrue(scoring.indexer.runIndexerCommand()
        .andThen(Commands.waitSeconds(3)).andThen(scoring.intake.retractWithRollersCommand())
        .andThen(Commands.waitSeconds(3)).andThen(scoring.intake.stopRollerCommand())
        .andThen(scoring.intake.stopExtendCommand()).andThen(scoring.indexer.stopCommand()));

    // Configure the trigger bindings
    configureBindings();
    configureDefaults();

    // Get autonomous ready
    AutoManager.prepare(scoring);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in
   * {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings() {
    RESET_HEADING.onTrue(swerve.runOnce(() -> swerve.resetHeading()));
    // SLOW_MODE.onTrue(swerve.runOnce(() -> swerve.setSlowMode(true)))
    // .onFalse(swerve.runOnce(() -> swerve.setSlowMode(false)));

    ALIGN_HEADING.onTrue(swerve.runOnce(() -> swerve.alignedHeading()));


    INTAKE_RUN.onTrue(scoring.intake.runIntakeRollersCommand())
        .onFalse(scoring.intake.stopRollerCommand());
    INTAKE_REVERSE.onTrue(scoring.intake.runReversedIntakeRollersCommand())
        .onFalse(scoring.intake.stopRollerCommand());

    INTAKE_EXTEND.onTrue(scoring.intake.extendIntakeCommand())
        .onFalse(scoring.intake.stopExtendCommand());
    INTAKE_RETRACT.onTrue(scoring.intake.retractIntakeCommand())
        .onFalse(scoring.intake.stopExtendCommand());
    INTAKE_RETRACT.onTrue(scoring.intake.retractIntakeCommand())
        .onFalse(scoring.intake.stopExtendCommand());
    RESET_EXTEND.onTrue(scoring.intake.resetExtenderCommand());

    // TODO: Test binding to put swerve wheels into an "X" pattern to resist being
    // pushed around.
    LOCK_WHEELS.whileTrue(swerve.runOnce(() -> swerve.brake()));

    // ENABLE_TRACKING start turret tracking and shooter wheels. It operates as a
    // toggle.
    ENABLE_TRACKING.onTrue(scoring.toggleTrackingCommand());

    SHOOT.onTrue(scoring.indexer.runIndexerCommand()).onFalse(scoring.indexer.stopCommand());

    RESET_TURRET.onTrue(scoring.turret.resetPosCommand());

    MANUAL_OVERRIDE.onTrue(scoring.overrideTrackingCommand());

    POSITIVE_TURRET_TESTING.onTrue(scoring.turret.basicTurretToPosCommand(90));

    REVERSE_TURRET_TESTING.onTrue(scoring.turret.basicTurretToPosCommand(-90));



    // SNAP_TO.onTrue(swerve.runOnce(() -> swerve.snapToAngle(new Rotation2d(90))));

    SHOOT_SQUEEZE.onTrue(scoring.indexer.runIndexerCommand().andThen(Commands.waitSeconds(3))
        .andThen(scoring.intake.retractWithRollersCommand()).andThen(Commands.waitSeconds(3))
        .andThen(scoring.intake.stopRollerCommand()).andThen(scoring.intake.stopExtendCommand())
        .andThen(scoring.indexer.stopCommand()));

  }

  /**
   * Place any default subsystem commands here
   */
  private void configureDefaults() {
    // Set the swerve's default command to drive with joysticks
    setDefaultCommand(swerve, swerve.run(() -> {
      swerve.drive(swerve.processJoystickInputs(-driverController.getLeftX(),
          -driverController.getLeftY(), -driverController.getRightX()));
    }).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return AutoManager.getAutonomousCommand();
  }

  public Vision getRightVision() {
    return visionRight;
  }

  public Vision getLeftVision() {
    return visionLeft;
  }

  public Vision getBackVision() {
    return visionBack;
  }

  /**
   * Sets the default command for a subsystem.
   * 
   * @param subsystem
   * @param command
   */
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
