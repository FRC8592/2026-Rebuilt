package frc.robot.commands.largecommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class LargeCommand extends Command {
    // protected static Swerve swerve;

    
    public static void addSubsystems(){
        // LargeCommand.swerve = swerve;
       
    }
    // Require at least one subsystem to be passed in
    public LargeCommand(Subsystem requirement1, Subsystem... moreRequirements){
        addRequirements(requirement1);
        addRequirements(moreRequirements);
    }
} 