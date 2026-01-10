// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.ArrayList;
import java.util.Set;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.commands.autonomous.autos.MoveOut;
import frc.robot.commands.proxies.*;
import frc.robot.subsystems.swerve.Swerve;

/**
 * General class for autonomous management (loading autos, sending the chooser, getting the
 * user-selected auto command, etc).
 */
public final class AutoManager {
    private static Swerve swerve;
    private static SendableChooser<AutoCommand> autoChooser;
    private static ArrayList<AutoCommand> autoCommands = new ArrayList<>();

    /**
     * Load all autos and broadcast the chooser.
     *<p>
     * * This is where programmers should add new autos.
     *
     * @apiNote This should be called on {@link Robot#robotInit()} only;
     * this function will have relatively long delays due to loading paths.
     */
    public static void prepare(){
        SmartDashboard.putNumber("Auto Delay", 0);
        autoCommands = new ArrayList<>();

        autoChooser = new SendableChooser<>();
        
        autoChooser.setDefaultOption("DEFAULT - No auto", new AutoCommand());
        for(AutoCommand c : autoCommands){
            autoChooser.addOption(
                c.getAutoName(), c
            );
        }
        Shuffleboard.getTab("Autonomous Config").add(autoChooser);
    }

    /**
     * Get the user-selected autonomous command as determined by {@link AutoManager#autoChooser}
     *
     * @return the command
     */
    public static Command getAutonomousCommand(){
        AutoCommand autoCommand = autoChooser.getSelected();
        return getAutonomousInitCommand().andThen(
            // If we don't keep this command from registering as composed,
            // the code will crash if we try to run an auto twice without
            // restarting robot code.
            new MultiComposableCommand(autoCommand)
        );
    }

    /**
     * Parallel command group that runs all subsystems' autonomous init commands.
     *
     * @return the command
     */
    private static Command getAutonomousInitCommand(){
        return new DeferredCommand(()->new WaitCommand(SmartDashboard.getNumber("Auto Delay", 0)), Set.of());
    }

    private AutoManager() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
