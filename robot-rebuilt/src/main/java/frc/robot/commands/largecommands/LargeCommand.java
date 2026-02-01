package frc.robot.commands.largecommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swerve.Swerve;

//mainly used for passing subsystems to the followPathCommand; 
//passing in the subsystems repeatedly for every followpathcommand is repetitive and unrealistic.
public abstract class LargeCommand extends Command {
    protected static Swerve swerve;

    public static void addSubsystems(Swerve swerve){
        LargeCommand.swerve = swerve;
    }
    
    public LargeCommand(Subsystem requirement1, Subsystem... moreRequirements){
        addRequirements(requirement1);
        // addRequirements(moreRequirements);

        
    }
} 