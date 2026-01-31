package frc.robot.commands.autonomous.autos;


import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class MoveCollect extends AutoCommand {
    public MoveCollect(){
        super(
            new FollowPathCommand(getChoreoTrajectory("MoveToBump"), Suppliers.isRedAlliance, "",1.0),




                    new FollowPathCommand(getChoreoTrajectory("CrossBump1"), Suppliers.isRedAlliance, "",0.25),

                    new FollowPathCommand(getChoreoTrajectory("RotateCollect"), Suppliers.isRedAlliance, "",1.0)




         );
    }
}
