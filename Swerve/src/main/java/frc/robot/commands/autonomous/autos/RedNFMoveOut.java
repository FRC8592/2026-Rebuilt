package frc.robot.commands.autonomous.autos;


import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Indexer;



public class RedNFMoveOut extends AutoCommand {
   public RedNFMoveOut(){
         super(
            new FollowPathCommand(getChoreoTrajectory("RedNFMoveOut"), Suppliers.isRedAlliance, "")
         );


   }


}


