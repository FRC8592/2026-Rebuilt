// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.swerve.CommandSwerveDrivetrain;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.Telemetry;
import frc.robot.subsystems.swerve.TunerConstants;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.vision.Vision;

import frc.robot.Constants.CONTROLLERS;
import frc.robot.Constants.*;
import frc.robot.commands.autonomous.AutoManager;
import frc.robot.subsystems.*;

import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;
/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

public class RobotContainer {
    private static final CommandXboxController driverController = new CommandXboxController(
        CONTROLLERS.DRIVER_PORT
    );
    
    private final Telemetry logger = new Telemetry(SWERVE.MAX_SPEED);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // The robot's subsystems
    private final Swerve swerve;
    private final OdometryUpdates odometryUpdates;
    private final Vision vision;
    private final Indexer indexer;

    private final Launcher testingLauncher;
    private double percentDashboard1;
    private double percentDashboard2;
    
    private final Trigger RESET_HEADING = driverController.back();
    // private final Trigger SLOW_MODE = driverController.rightBumper();
    private final Trigger LAUNCH = driverController.rightTrigger();
    private Trigger RUN = driverController.rightBumper();
    private final Trigger TESTINGINTAKEBOTTOMBUTTON = driverController.leftBumper();

    //private final Trigger TESTINGINTAKEBUTTON = driverController.rightTrigger();
    private final Trigger TESTINGINTAKESIDEBUTTON = driverController.leftTrigger();
    private Intake testingIntake;
    

    /**
     * Create the robot container. This creates and configures subsystems, sets
     * up button bindings, and prepares for autonomous.
     */
    public RobotContainer() {
        swerve = new Swerve(drivetrain);
        vision = new Vision(VISION.CAMERA_NAME, VISION.CAMERA_OFFSETS);
        odometryUpdates = new OdometryUpdates(vision, swerve);
        testingLauncher = new Launcher();
        indexer = new Indexer();
        testingIntake = new Intake();

        configureBindings();
        configureDefaults();
        
        AutoManager.prepare();
    }

    /**
     * Configure default commands for the subsystems
     */
    private void configureDefaults(){
        // Set the swerve's default command to drive with joysticks

        setDefaultCommand(swerve, swerve.run(() -> {
            swerve.drive(swerve.processJoystickInputs(
                -driverController.getLeftX(),
                -driverController.getLeftY(),
                -driverController.getRightX()
            ));
        }).withInterruptBehavior(InterruptionBehavior.kCancelSelf));
    }

    /**
     * Configure all button bindings
     */
    private void configureBindings() {
        // SLOW_MODE.onTrue(Commands.runOnce(() -> swerve.setSlowMode(true)).ignoringDisable(true))
                //  .onFalse(Commands.runOnce(() -> swerve.setSlowMode(false)).ignoringDisable(true));

        RESET_HEADING.onTrue(swerve.runOnce(() -> swerve.resetHeading()));

        double percentDerived1 = SmartDashboard.getNumber("bottom_launcher_motor",percentDashboard1);
        double percentDerived2 = SmartDashboard.getNumber("top_launcher_motor",percentDashboard2);
        //Try and print the values
        System.out.println("PercentDashboard1 " + percentDashboard1);
        System.out.println("PercentDashboard2 " + percentDashboard2);
        LAUNCH.whileTrue(new DeferredCommand(() -> testingLauncher.setLauncherCommand(SmartDashboard.getNumber("bottom_launcher_motor", 0.4), SmartDashboard.getNumber("top_launcher_motor", 0.4)), Set.of(testingLauncher))).onFalse(testingLauncher.stopLauncherCommand());

        RUN.onTrue(
          new DeferredCommand(() -> indexer.setMotorPercentOutputCommand(.5), Set.of(indexer))
        ).onFalse(indexer.stopMotorCommand());

        driverController.a().onTrue(
            new DeferredCommand(() -> indexer.setMotorPercentOutputCommand(0, .5), Set.of(indexer))
        ).onFalse(indexer.stopMotorCommand(0));

        driverController.b().onTrue(
            new DeferredCommand(() -> indexer.setMotorPercentOutputCommand(2, .5), Set.of(indexer))
        ).onFalse(indexer.stopMotorCommand(2));

        driverController.y().onTrue(
            new DeferredCommand(() -> indexer.setMotorPercentOutputCommand(3, .5), Set.of(indexer))
        ).onFalse(indexer.stopMotorCommand(3));

        TESTINGINTAKESIDEBUTTON.onTrue(new DeferredCommand(() -> testingIntake.setIntakeSideCommand(0.75), Set.of(testingIntake))).onFalse(testingIntake.stopIntakeSideCommand());
        //TESTINGINTAKEBOTTOMBUTTON.onTrue(new DeferredCommand(() -> testingIntake.setIntakeBottomCommand(0.5), Set.of(testingIntake))).onFalse(testingIntake.stopIntakeBottomCommand());
        //TESTINGPIVOTINTAKEBUTTON.onTrue(new DeferredCommand(() ->testingIntake.setIntakeCommand(testingIntake.accessPivotIntakeMotor(),0.5), Set.of(testingIntake))).onFalse(testingIntake.stopIntakeCommand(testingIntake.accessPivotIntakeMotor()));
  
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
     * Set the default command of a subsystem (what to run if no other command requiring it is running).
     * <p> NOTE: all subsystems also have a setDefaultCommand method; this version includes a check for
     * default commands that cancel incoming commands that require the subsystem. Unless you're sure
     * of what you're doing, you should use this one.
     *
     * @param subsystem the subsystem to apply the default command to
     * @param command to command to set as default
     */
    private void setDefaultCommand(SubsystemBase subsystem, Command command){
        if(command.getInterruptionBehavior() == InterruptionBehavior.kCancelSelf){
            subsystem.setDefaultCommand(command);
        }
        else{
            //If you want to force-allow setting a cancel-incoming default command, directly call `subsystem.setDefaultCommand()` instead
            throw new UnsupportedOperationException("Can't set a default command that cancels incoming!");
        }
    }

}
