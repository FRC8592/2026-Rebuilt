package frc.robot.commands.autonomous.autos;


import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class Testing extends AutoCommand {
    public Testing(){
        super(
            new FollowPathCommand(getChoreoTrajectory("Testing"), Suppliers.isRedAlliance, "")



         );
    }
}
