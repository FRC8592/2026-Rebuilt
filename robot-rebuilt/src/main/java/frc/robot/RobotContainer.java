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

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private static final CommandXboxController driverController = new CommandXboxController(CONTROLLERS.DRIVER_PORT);

  // robot subsystems
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final Swerve swerve;
  public final Shooter shooter;
  public final Intake intake;
  public final Indexer indexer;

  private final Trigger runIntake = driverController.rightBumper();
  private final Trigger runIndexer = driverController.leftBumper();
  private final Trigger RESET_HEADING = driverController.back();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerve = new Swerve(drivetrain);
    shooter = new Shooter();
    intake = new Intake();
    indexer = new Indexer();

    //Register commands that are going to be used in auto here BEFORE initializing autos
    //i.e. NamedCommands.registerCommand("commandName", exampleSubsystem.exampleCommand());
    NamedCommands.registerCommand("runIntake", intake.runAtSpeedCommand());
    NamedCommands.registerCommand("stopIntake", intake.stopCommand());

    // new EventTrigger(null)
    
    // Configure the trigger bindings
    configureBindings();
    configureDefaults();
    passSubsystems();

    AutoManager.prepare();
  }

    private void passSubsystems(){
        LargeCommand.addSubsystems(swerve);
        AutoCommand.addSubsystems(swerve);
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
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    runIntake.whileTrue(intake.runAtSpeedCommand()).onFalse(intake.stopCommand());
    runIndexer.whileTrue(indexer.runAtSpeedCommand()).onFalse(indexer.stopCommand());
    RESET_HEADING.onTrue(swerve.runOnce(() -> swerve.resetHeading()));

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
