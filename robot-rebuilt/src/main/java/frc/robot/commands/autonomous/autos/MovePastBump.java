package frc.robot.commands.autonomous.autos;


// import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class MovePastBump extends AutoCommand {
    public MovePastBump(){
        super(
            new FollowPathCommand(getChoreoTrajectory("MovePastBump"))
         );
    }
}
