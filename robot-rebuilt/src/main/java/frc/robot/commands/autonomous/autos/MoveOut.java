package frc.robot.commands.autonomous.autos;


import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class MoveOut extends AutoCommand {
    public MoveOut(){
        super(
            new FollowPathCommand(getChoreoTrajectory("MoveToBump"), Suppliers.isRedAlliance, ""),




                    new FollowPathCommand(getChoreoTrajectory("CrossBump1"), Suppliers.isRedAlliance, "")




         );
    }
}
