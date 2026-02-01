package frc.robot.commands.autonomous.autos;


// import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class MoveCollectReturn extends AutoCommand {
    public MoveCollectReturn(){
        super(
            new FollowPathCommand(getChoreoTrajectory("MoveCollectReturn"))
         );
    }
}
