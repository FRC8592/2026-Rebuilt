package frc.robot.commands.autonomous.autos;

import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;

public class RedNFMoveOut extends AutoCommand {
   public RedNFMoveOut(){
         super(
            new FollowPathCommand(getChoreoTrajectory("RedMoveOut"), Suppliers.isRedAlliance, "")
         );

   }

}


