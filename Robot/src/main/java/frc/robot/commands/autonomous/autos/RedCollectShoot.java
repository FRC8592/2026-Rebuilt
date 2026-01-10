package frc.robot.commands.autonomous.autos;


import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Indexer;


public class RedCollectShoot extends AutoCommand {
  public RedCollectShoot(String color, Launcher launcher, Indexer indexer){
      super(
        new FollowPathCommand(getChoreoTrajectory("Shoot"), Suppliers.isRedAlliance, "")
        .andThen(launcher.setLauncherCommand(
         0.44, 0.30   // bottom %, top %
     ).withTimeout(2.0))  // spin up time (tune this)
     .andThen(
         launcher
             .setLauncherCommand(0.44, 0.30)   // keep launcher running
             .alongWith(
                 // feed with indexer 
                 indexer.setMotorPercentOutputCommand(1)
             )
             .withTimeout(2.0)                 // feed time (tune this)
     )
     // 4) Stop launcher after feeding
     .andThen(
         launcher.stopLauncherCommand()
     ).andThen(
       new FollowPathCommand(getChoreoTrajectory("CollectFirst"), Suppliers.isRedAlliance, "")),
       /*
        * .andThen(Intake Command until Indexer detects 1 Lumen in its system))
        * .andThen(Indexer Command until their conditional is satisfied)
       */

       new FollowPathCommand(getChoreoTrajectory("RotateFirst"), Suppliers.isRedAlliance, ""),
         /*
         * No need to do anything here
         */

       new FollowPathCommand(getChoreoTrajectory("CollectSecond"), Suppliers.isRedAlliance, ""),
       /*
        * .andThen(Intake Command until Indeder detects another Lumen in its system)
        * .andThen(Indexer Command until their conditional is satisfied)
        *
        */
       
       new FollowPathCommand(getChoreoTrajectory("CollectThird"), Suppliers.isRedAlliance, ""),
               /*
        * .andThen(Intake Command until Indeder detects another Lumen in its system)
        * .andThen(Indexer Command until their conditional is satisfied)
        *
        */


       new FollowPathCommand(getChoreoTrajectory("ShootRest"), Suppliers.isRedAlliance, ""),
               /*
        * .andThen(Intake Stow)
        * .andThen(Launcher Shoot 3, Steep Angle Shot)
        *
        */
        new FollowPathCommand(getChoreoTrajectory("MoveOutAfterLine"), Suppliers.isRedAlliance, ""));
        /*
 * Nothing really needed here
 *
 */



   }


}


