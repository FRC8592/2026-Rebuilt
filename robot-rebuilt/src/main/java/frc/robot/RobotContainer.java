// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;


import java.nio.file.OpenOption;
import java.util.Set;

import javax.sound.sampled.SourceDataLine;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
// import frc.robot.subsystems.swerve.Swerve;
// import frc.robot.subsystems.swerve.Telemetry;
// import frc.robot.subsystems.swerve.TunerConstants;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.*;
// import frc.robot.commands.autonomous.AutoManager;
import frc.robot.subsystems.*;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here... 

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private static final CommandXboxController driverController = new CommandXboxController(CONTROLLERS.DRIVER_PORT);
  private static final CommandXboxController operatorController = new CommandXboxController(CONTROLLERS.OPERATOR_PORT);

  
  private  final Trigger intakeTrigger = driverController.leftBumper();
  private final Trigger outtakeTrigger = driverController.rightBumper();

  private final Intake intake;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    intake = new Intake (); 

    // Configure the trigger bindings
    configureBindings();
  }

//  private void configureDefaults() {
//         // Set the swerve's default command to drive with joysticks

//         setDefaultCommand(swerve, swerve.run(() -> {
//             swerve.drive(swerve.processJoystickInputs(
//                     -driverController.getLeftX(),
//                     -driverController.getLeftY(),
//                     -driverController.getRightX()));
//         }).withInterruptBehavior(InterruptionBehavior.kCancelSelf));

//         indexer.setDefaultCommand(
//                 new RunCommand(() -> indexer.autoIndex(), indexer));

//     }
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    intakeTrigger.whileTrue(intake.setIntakeCommand(1.0)).onFalse (intake.stopIntakeCommand());
    outtakeTrigger.whileTrue(intake.setOuttakeCommand(-1.0)).onFalse(intake.stopIntakeCommand());
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    //m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
//   public Command getAutonomousCommand() {
//     //An example command will be run in autonomous
//     //return Autos.exampleAuto(m_exampleSubsystem);
//   }
// }

}