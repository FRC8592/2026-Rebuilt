// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.autonomous.AutoManager;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.OdometryUpdates;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.Scoring;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private static final CommandXboxController driverController = new CommandXboxController(CONTROLLERS.DRIVER_PORT);

  // robot subsystems
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Swerve swerve;
  public final Vision visionBack;
  public final Vision visionSide; 
  public final OdometryUpdates odometryUpdatesBack;
  public final OdometryUpdates odometryUpdatesSide; 
  public final Scoring scoring;

  private final Trigger RESET_HEADING = driverController.back();
  private final Trigger SLOW_MODE = driverController.leftTrigger();

  // private final Trigger RUN_INDEXER = driverController.a();
  // private final Trigger RUN_SHOOTER = driverController.b();
  // private final Trigger INTAKE_RUN = driverController.leftBumper();
  // private final Trigger TURRET_TEST = driverController.x();

  // //used in sysId testing
  // private final Trigger QUASI_FORWARD = driverController.a();
  // private final Trigger QUASI_REVERSE = driverController.y();
  // private final Trigger DYNAMIC_FORWARD = driverController.b();
  // private final Trigger DYNAMIC_REVERSE = driverController.x();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve = new Swerve(drivetrain);
    visionBack = new Vision(VISION.CAMERA_NAME_BACK, VISION.CAMERA_OFFSETS_BACK);
    visionSide = new Vision(VISION.CAMERA_NAME_SIDE, VISION.CAMERA_OFFSETS_SIDE);
    odometryUpdatesBack = new OdometryUpdates(visionBack, swerve);
    odometryUpdatesSide = new OdometryUpdates(visionSide, swerve); 
    
    scoring = new Scoring(swerve);
    
    // Configure the trigger bindings
    configureBindings();
    configureDefaults();

    AutoManager.prepare();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    RESET_HEADING.onTrue(swerve.runOnce(() -> swerve.resetHeading()));
    SLOW_MODE.onTrue(swerve.runOnce(() -> swerve.setSlowMode(true)))
             .onFalse(swerve.runOnce(() -> swerve.setSlowMode(false)));
    
    // RUN_INDEXER.onTrue(indexer.runIndexerCommand()).onFalse(indexer.stopCommand());
    // INTAKE_RUN.onTrue(intake.runAtSpeedRightCommand()).onFalse(intake.stopRollerCommand());
    // RUN_SHOOTER.onTrue(shooter.runAtSpeedCommand()).onFalse(shooter.stopShooterCommand());
    // TURRET_TEST.onTrue(scoring.autoTurretCommand()).onFalse(turret.stopTurretCommand());
  }

  private void configureDefaults() {
        // Set the swerve's default command to drive with joysticks
        setDefaultCommand(swerve, swerve.run(() -> {
            swerve.drive(swerve.processJoystickInputs(
                    -driverController.getLeftX(),
                    -driverController.getLeftY(),
                    -driverController.getRightX()));
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
