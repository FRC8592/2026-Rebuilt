// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

// import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
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
        scoring = scr;

        pathPlannerAutos = AutoBuilder.buildAutoChooser();
       /* 
        try {
            PathPlannerPath halfRight = PathPlannerPath.fromPathFile("HALF LEFT").mirrorPath();
            Command halfMirroredAuto = AutoBuilder.followPath(halfRight);

            pathPlannerAutos.addOption("ONE Half Right", halfMirroredAuto);

            System.out.println("Added pathplanner right auto");
        } catch (Exception e) {
            DriverStation.reportError("Failed to load mirrored path Half Left: " + e.getMessage(),
                    e.getStackTrace());

            System.out.println("Exception in adding pathplanner right auto");
        }

        try {
            PathPlannerPath halfRightTwo = PathPlannerPath.fromPathFile("HALF LEFT");
            Command halfDoubleFirstCommand = AutoBuilder.followPath(halfRightTwo);

            PathPlannerPath halfRightTwoSecond = PathPlannerPath.fromPathFile("Half Left Second Swipe");
            Command halfDoubleSecondCommand = AutoBuilder.followPath(halfRightTwoSecond);

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

            pathPlannerAutos.addOption("Double Half Right", doubleSwipeAuto);

        } catch (Exception e) {
            DriverStation.reportError("Failed to load mirrored path Half Left doubl: " + e.getMessage(),
                    e.getStackTrace());

            System.out.println("Exception in adding pathplanner double right auto");
        }
*/
        Shuffleboard.getTab("Autonomous Config").add(pathPlannerAutos);
        SmartDashboard.putData("Auto Chooser", pathPlannerAutos);

    }

    /**
     * Get the user-selected autonomous command as determined by {@link AutoManager#autoChooser}
     *
     * @return the command
     */
    public static Command getAutonomousCommand() {
        return pathPlannerAutos.getSelected();
    }

    private AutoManager() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
