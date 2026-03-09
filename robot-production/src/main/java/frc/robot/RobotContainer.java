// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.*;
import frc.robot.commands.autonomous.AutoManager;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.LEDs;
import frc.robot.subsystems.OdometryUpdates;
import frc.robot.subsystems.swerve.TunerConstants;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.Scoring;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Arrays;
import java.util.Collection;
import java.util.Set;

public class RobotContainer {
  private static final CommandXboxController driverController = new CommandXboxController(CONTROLLERS.DRIVER_PORT);
  private static final CommandXboxController operatorController = new CommandXboxController(CONTROLLERS.OPERATOR_PORT);

  // Robot subsystems
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Swerve swerve;
  private final Vision visionBack;
  private final Vision visionSide; 
  private final OdometryUpdates odometryUpdatesBack;
  private final OdometryUpdates odometryUpdatesSide;
  public final Scoring scoring;
  public final LEDs leds; 

  //
  // Driver Controls
  //
  private final Trigger RESET_HEADING = driverController.back();
  private final Trigger SLOW_MODE = driverController.leftTrigger();
  private final Trigger INTAKE_RUN = driverController.rightTrigger();
  // TODO: Change Intake Extend Binding
  private final Trigger INTAKE_EXTEND = operatorController.y();
  private final Trigger LOCK_WHEELS = driverController.x();

  // private final Trigger SNAP_TO = driverController.povUp();

  //
  // Operator Controls
  //
  private final Trigger RESET_TURRET = driverController.a();
  private final Trigger ENABLE_TRACKING = operatorController.leftTrigger();
  private final Trigger SHOOT = operatorController.rightTrigger();

  //private final Trigger TURRET_TEST = operatorController.x();
  //private final Trigger TURRET_TEST_BACK = operatorController.a();

  private final Trigger RESET_EXTEND = operatorController.b();


  //
  // Controls for running sysId tests
  //
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
    scoring = new Scoring(swerve);
    visionBack = new Vision(VISION.CAMERA_NAME_BACK, VISION.CAMERA_OFFSETS_BACK);
    visionSide = new Vision(VISION.CAMERA_NAME_SIDE, VISION.CAMERA_OFFSETS_SIDE);
    odometryUpdatesBack = new OdometryUpdates(visionBack, swerve);
    odometryUpdatesSide = new OdometryUpdates(visionSide, swerve); 
    
    //
    // Configure the trigger bindings
    //
    configureBindings();
    configureDefaults();

    //
    // Get autonomous ready
    //
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

    // SLOW_MODE.onTrue(swerve.runOnce(() -> swerve.setSlowMode(true)))
    //          .onFalse(swerve.runOnce(() -> swerve.setSlowMode(false)));

    INTAKE_RUN.onTrue(scoring.intake.runAtSpeedIntakeCommand()).onFalse(scoring.intake.stopRollerCommand());

    //TURRET_TEST.onTrue(scoring.turret.basicTurretTestingCommand(45)).onFalse(scoring.turret.stopTurretCommand());

    //TURRET_TEST_BACK.onTrue(scoring.turret.basicTurretTestingCommand(-45)).onFalse(scoring.turret.stopTurretCommand());

    //RESET_EXTEND.onTrue(scoring.intake.resetExtenderCommand());


    //INTAKE_EXTEND.onTrue(scoring.intake.runExtendCommand()).onFalse(scoring.intake.stopExtendCommand());

    // TODO: Test binding to put swerve wheels into an "X" pattern to resist being pushed around.
    LOCK_WHEELS.whileTrue(swerve.runOnce(() -> swerve.brake()));

    // ENABLE_TRACKING start turret tracking and shooter wheels.  It operates as a toggle.
    ENABLE_TRACKING.onTrue(scoring.toggleTrackingCommand());

    SHOOT.onTrue(scoring.indexer.runIndexerCommand()).onFalse(scoring.indexer.stopCommand());

    //RESET_TURRET.onTrue(scoring.turret.resetPosCommand());

    // SNAP_TO.onTrue(swerve.runOnce(() -> swerve.snapToAngle(new Rotation2d(90))));
  }

  /**
   * Place any default subsystem commands here
   */
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

  public Vision getSideVision(){
    return visionSide; 
  }

  public Vision getBackVision(){
    return visionBack; 
  }
  
  /**
   * Sets the default command for a subsystem. 
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
