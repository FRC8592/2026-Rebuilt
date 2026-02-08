package frc.robot.commands.autonomous.autos;


// import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class MoveCollect extends AutoCommand {
    public MoveCollect(){
        super(
            new FollowPathCommand(getChoreoTrajectory("MoveToBump")),
            new FollowPathCommand(getChoreoTrajectory("CrossBump1")),
            new FollowPathCommand(getChoreoTrajectory("RotateCollect"))
         );
    }
}
