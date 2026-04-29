// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import java.util.Set;

import com.pathplanner.lib.auto.AutoBuilder;

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
