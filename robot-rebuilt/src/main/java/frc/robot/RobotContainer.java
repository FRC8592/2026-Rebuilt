// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.autonomous.AutoManager;
import frc.robot.commands.largecommands.LargeCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.TunerConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Indexer; 
import frc.robot.subsystems.Climb;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {

  


  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(0);

  // robot subsystems
  //public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  //private final Swerve swerve;
  // public final Shooter shooter;
  // public final Intake intake;
  // public final Indexer indexer;
  public final Climb climb; 

  private final Trigger runIntake = driverController.rightBumper();
  private final Trigger runClimber = driverController. rightTrigger();
  private final Trigger reverseClimber = driverController.leftTrigger();
  private final Trigger runIndexer = driverController.leftBumper();
  private final Trigger RESET_HEADING = driverController.back();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // swerve = new Swerve(drivetrain);
    // shooter = new Shooter();
    // intake = new Intake();
    // indexer = new Indexer();
    climb = new Climb();
    
    // Configure the trigger bindings
    configureBindings();
    //configureDefaults();
    
    // LargeCommand.addSubsystems(swerve);
    // AutoCommand.addSubsystems(swerve);

    // passSubsystems();
    // AutoManager.prepare();
  }

    // private void passSubsystems(){
    //     LargeCommand.addSubsystems(swerve);
    // }

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
    runClimber.whileTrue(climb.setClimbCommand(4)).onFalse(climb.stopCommand());
    reverseClimber.whileTrue(climb.setReverseCommand(-4)).onFalse(climb.stopCommand());
    //runIntake.whileTrue(intake.runAtSpeedCommand()).onFalse(intake.stopCommand());
    //runIndexer.whileTrue(indexer.runAtSpeedCommand()).onFalse(indexer.stopCommand());
    //RESET_HEADING.onTrue(swerve.runOnce(() -> swerve.resetHeading()));
    

  }

  // private void configureDefaults() {
  //       // Set the swerve's default command to drive with joysticks

  //       setDefaultCommand(swerve, swerve.run(() -> {
  //           swerve.drive(swerve.processJoystickInputs(
  //                   -driverController.getLeftX(),
  //                   -driverController.getLeftY(),
  //                   -driverController.getRightX()));
  //       }).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

  //   }

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
