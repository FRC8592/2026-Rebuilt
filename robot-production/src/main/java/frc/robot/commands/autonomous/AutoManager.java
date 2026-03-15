// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

// import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Robot;
import frc.robot.subsystems.Scoring;

/**
 * General class for autonomous management (loading autos, sending the chooser, getting the
 * user-selected auto command, etc).
 */
public final class AutoManager {
    private static SendableChooser<Command> pathPlannerAutos;

    /**
     * Load all autos and broadcast the chooser.
     * @apiNote This should be called on {@link Robot#robotInit()} only;
     * this function will have relatively long delays due to loading paths.
     */
    public static void prepare(Scoring scoring){
        pathPlannerAutos = AutoBuilder.buildAutoChooser();
        try {
            PathPlannerPath halfLeftPath = PathPlannerPath.fromPathFile("Half Left");
            PathPlannerPath HalfRight = halfLeftPath.mirrorPath();
            pathPlannerAutos.addOption("Half Right", AutoBuilder.followPath(HalfRight));
        } 
        catch (Exception e) {
            DriverStation.reportError("Failed to load mirrored path Half Left: " + e.getMessage(), e.getStackTrace());
        }
        try {
            PathPlannerPath halfLeftPath = PathPlannerPath.fromPathFile("Half Left");
             PathPlannerPath DepotPt1Path = PathPlannerPath.fromPathFile("Depot Pt1");
                          PathPlannerPath DepotPt2Path = PathPlannerPath.fromPathFile("Depot Pt2");


            PathPlannerPath depot = halfLeftPath.mirrorPath();
            pathPlannerAutos.addOption("Depot", AutoBuilder.followPath(depot).andThen(AutoBuilder.followPath(DepotPt1Path)).andThen(Commands.waitSeconds(2.0)).andThen(AutoBuilder.followPath(DepotPt2Path)).andThen(scoring.indexer.runIndexerCommand()));
        } 
        catch (Exception e) {
            DriverStation.reportError("Failed to load Depot: " + e.getMessage(), e.getStackTrace());
        }
        Shuffleboard.getTab("Autonomous Config").add(pathPlannerAutos);
        
    }

    /**
     * Get the user-selected autonomous command as determined by {@link AutoManager#autoChooser}
     *
     * @return the command
     */
    public static Command getAutonomousCommand(){
        return pathPlannerAutos.getSelected();
    }

    private AutoManager() {
        throw new UnsupportedOperationException("This is a utility class!");
    }
}
