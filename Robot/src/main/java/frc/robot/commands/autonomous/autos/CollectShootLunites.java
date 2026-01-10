package frc.robot.commands.autonomous.autos;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;


public class CollectShootLunites extends AutoCommand {
    public CollectShootLunites(String color){
       super(
        new FollowPathCommand(getChoreoTrajectory("AC_TurnToIntake"), Suppliers.isRedAlliance, ""),
        /*
         * .andThen(Intake Command until Indexer detects 1 Lumen in its system))
         * .andThen(Indexer Command until their conditional is satisfied)
        */
 
        new FollowPathCommand(getChoreoTrajectory("AC_IntakeFirst"), Suppliers.isRedAlliance, ""),
          /*
          * No need to do anything here
          */
 
        new FollowPathCommand(getChoreoTrajectory("AC_IntakeSecond"), Suppliers.isRedAlliance, ""),
        /*
         * .andThen(Intake Command until Indeder detects another Lumen in its system)
         * .andThen(Indexer Command until their conditional is satisfied)
         *
         */
        
        new FollowPathCommand(getChoreoTrajectory("AC_IntakeThird"), Suppliers.isRedAlliance, ""),
                /*
         * .andThen(Intake Command until Indeder detects another Lumen in its system)
         * .andThen(Indexer Command until their conditional is satisfied)
         *
         */
 
 
        new FollowPathCommand(getChoreoTrajectory("AC_GoToShoot"), Suppliers.isRedAlliance, ""),
                /*
         * .andThen(Intake Stow)
         * .andThen(Launcher Shoot 3, Regular Shot)
         *
         */
         new FollowPathCommand(getChoreoTrajectory("AC_MoveOut"), Suppliers.isRedAlliance, ""));
         /*
  * Nothing really needed here
  *
  */
 
 
 
    }
 
 
 }
