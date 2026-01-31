package frc.robot.commands.autonomous.autos.BunnyBotAutos;

import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.FollowPathCommand;

public class BlueMoveOut extends AutoCommand {
    public BlueMoveOut(){
        super(
            new FollowPathCommand(getChoreoTrajectory("BlueMoveOut"))
         );
    }
}
