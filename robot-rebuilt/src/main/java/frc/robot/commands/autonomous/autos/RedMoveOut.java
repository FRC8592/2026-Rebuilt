package frc.robot.commands.autonomous.autos;


import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;




public class RedMoveOut extends AutoCommand {
   
   public RedMoveOut(){
     
                  Rotation2d rotation=new Rotation2d(0);

      Pose2d intialPose= new Pose2d(0,0,rotation);


         swerve.setKnownOdometryPose(intialPose);
         callme();


   

         

         

   }
public static void driveCommand  (){

 super(
            
            new FollowPathCommand(getChoreoTrajectory("RedMoveOut"), Suppliers.isRedAlliance, "")
         );

   }

}




