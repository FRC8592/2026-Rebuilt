// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.subsystems.Scoring;

/**
 * General class for autonomous management (loading autos, sending the chooser, getting the
 * user-selected auto command, etc).
 */
public final class AutoManager {
    private static SendableChooser<Command> pathPlannerAutos;
    private static Scoring scoring;

    /**
     * Load all autos and broadcast the chooser.
     * 
     * @apiNote This should be called on {@link Robot#robotInit()} only; this function will have
     *          relatively long delays due to loading paths.
     */
    public static void prepare(Scoring scr) {
        SmartDashboard.putNumber("Auto Delay", 0);
        scoring = scr;

        pathPlannerAutos = AutoBuilder.buildAutoChooser();

        Shuffleboard.getTab("Autonomous Config").add(pathPlannerAutos);
        SmartDashboard.putData("Auto Chooser", pathPlannerAutos);

        try {
            // Mirrored the left path to create the right path
            PathPlannerPath halfLeft = PathPlannerPath.fromPathFile("HALF LEFT");
            PathPlannerPath halfRight = halfLeft.mirrorPath();
            
            // Made the right path command based on the mirrored path
            Command halfMirroredAuto = AutoBuilder.followPath(halfRight); 
            
            // Added the right path command to the auto chooser
            pathPlannerAutos.addOption("ONE Half Right", halfMirroredAuto); 

             // Added the left path command to the auto chooser
            pathPlannerAutos.addOption("ONE Half Left", AutoBuilder.followPath(halfLeft));

            // Outputed to the terminal if the right path command was added successfully
            System.out.println("Added pathplanner right auto"); 
        } catch (Exception e) { 
            // Report any errors in openning or loading the file to the driver station and output to the terminal
            DriverStation.reportError("Failed to load mirrored path Half Left: " + e.getMessage(),
                    e.getStackTrace());

            System.out.println("Exception in adding pathplanner right auto");
        }

        try {
            PathPlannerPath halfRightTwo = PathPlannerPath.fromPathFile("HALF LEFT");

            // Create the command for the first half of the double swipe auto
            Command halfDoubleFirstCommand = AutoBuilder.followPath(halfRightTwo);

            PathPlannerPath halfRightTwoSecond = PathPlannerPath.fromPathFile("Half Left Second Swipe");
            // Create the command for the second half of the double swipe auto
            Command halfDoubleSecondCommand = AutoBuilder.followPath(halfRightTwoSecond);

            // Combine multiple path commands with wait time
            Command doubleSwipeAuto = halfDoubleFirstCommand.andThen(scoring.toggleTrackingCommand())
                                                        .andThen(new WaitCommand(0.9))
                                                        .andThen(scoring.indexer.runIndexerCommand())
                                                        .andThen(new WaitCommand(3))
                                                        .andThen(scoring.indexer.stopCommand())
                                                        .andThen(scoring.toggleTrackingCommand())
                                                        .andThen(halfDoubleSecondCommand)
                                                        .andThen(scoring.toggleTrackingCommand())
                                                        .andThen(new WaitCommand(0.9))
                                                        .andThen(scoring.indexer.runIndexerCommand())
                                                        .andThen(new WaitCommand(3))
                                                        .andThen(scoring.indexer.stopCommand())
                                                        .andThen(scoring.toggleTrackingCommand());

            // Added the double swipe auto command to the auto chooser
            pathPlannerAutos.addOption("Double Half Right", doubleSwipeAuto);

        } catch (Exception e) {
            // Catch and report any errors that occured
            DriverStation.reportError("Failed to load mirrored path Half Left double: " + e.getMessage(),
                    e.getStackTrace());

            System.out.println("Exception in adding pathplanner double right auto");
        }

    }

    /**
     * Get the user-selected autonomous command as determined by {@link AutoManager#autoChooser}
     *
     * @return the command
     */
    public static Command getAutonomousCommand() {
        return new DeferredCommand(() -> new WaitCommand(SmartDashboard.getNumber("Auto Delay", 0)), Set.of())
        .andThen(pathPlannerAutos.getSelected());
    }

    private AutoManager() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
