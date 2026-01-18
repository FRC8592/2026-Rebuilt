package frc.robot.commands.autonomous.autos;


import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;




public class RedMoveOut extends AutoCommand {
   public RedMoveOut(){
         super(
            new FollowPathCommand(getChoreoTrajectory("RedMoveOut"), Suppliers.isRedAlliance, "")
         );


   }


}


