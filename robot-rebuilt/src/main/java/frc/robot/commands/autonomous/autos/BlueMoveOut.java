package frc.robot.commands.autonomous.autos;


import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class BlueMoveOut extends AutoCommand {
    public BlueMoveOut(){
        super(
            new FollowPathCommand(getChoreoTrajectory("BlueMoveOut"), Suppliers.isRedAlliance, "")
         );
    }
}
