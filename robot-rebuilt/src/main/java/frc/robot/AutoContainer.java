package frc.robot;

import java.util.ArrayList;
import java.util.Set;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.autonomous.autos.BlueMoveOut;
import frc.robot.commands.largecommands.LargeCommand;
import frc.robot.commands.proxies.MultiComposableCommand;
import frc.robot.commands.proxies.*;

import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.Robot;


public class AutoContainer {
    private static SendableChooser<AutoCommand> autoChooser;
    private static ArrayList<AutoCommand> autoCommands = new ArrayList<>();
    
    private final Swerve swerve;
    public final Shooter shooter;
    public final Intake intake;
    public final Indexer indexer;


    public AutoContainer(Swerve swerve, Shooter shooter, Intake intake, Indexer indexer){
        this.swerve = swerve;
        this.shooter = shooter;
        this.intake = intake;
        this.indexer = indexer;

        LargeCommand.addSubsystems(swerve);
        AutoCommand.addSubsystems(swerve);
        prepare();
    }

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
        autoCommands.add(new BlueMoveOut());

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
    public Command getAutonomousCommand(){
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
    private Command getAutonomousInitCommand(){
        
    }

    /**
    * Use this to pass the autonomous command to the main {@link Robot} class.
    *
    * @return the command to run in autonomous
    */
    // public Command getAutonomousCommand() {
    //     return new DeferredCommand(()->new WaitCommand(SmartDashboard.getNumber("Auto Delay", 0)), Set.of());
    // }
        
}
