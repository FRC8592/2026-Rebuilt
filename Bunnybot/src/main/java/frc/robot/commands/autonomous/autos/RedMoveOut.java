package frc.robot.commands.autonomous.autos;


import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Indexer;



public class RedMoveOut extends AutoCommand {
   public RedMoveOut(){
         super(
            new FollowPathCommand(getChoreoTrajectory("RedMoveOut"), Suppliers.isRedAlliance, "")
         );


   }


}


