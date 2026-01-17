package frc.robot.commands.autonomous.autos;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// public class RedShootLowMove extends AutoCommand {
//    public RedShootLowMove(String color, Launcher launcher, Indexer indexer){
//       super(
        
//        new FollowPathCommand(getChoreoTrajectory("RA_RedShoot"), Suppliers.isRedAlliance, "")
//        .andThen(launcher.setLauncherCommand(
//         0.23, 0.23   // bottom %, top % //CHANGE THIS FOR LOW GOAL
//     ).withTimeout(2.0))  // spin up time (tune this)
//     .andThen(
//         launcher
//             .setLauncherCommand(0.44, 0.30)   // keep launcher running
//             .alongWith(
//                 // feed with indexer 
//                 indexer.setMotorPercentOutputCommand(1)
//             )
//             .withTimeout(2.0)                 // feed time (tune this)
//     )
//     // 4) Stop launcher after feeding
//     .andThen(
//         launcher.stopLauncherCommand()
//     ).andThen(
//     // 5) Drive out of the zone

//        new FollowPathCommand(getChoreoTrajectory("RA_RedMoveOut"), Suppliers.isRedAlliance, "")));


//    }

// }

//This autonomous starts right in front of the AprilTag on the Cosmic Converter
public class RedShootLowMove extends AutoCommand {
    public RedShootLowMove(){
        super(
            //start by shooting two low lunites
            launcher.setLauncherCommand(0.23, 0.23) //use LOW
            .andThen(new WaitUntilCommand(0.5))
            .andThen(new RunCommand(() -> indexer.shoot(1))).withTimeout(2)

            .andThen(new FollowPathCommand(getChoreoTrajectory("RedMoveOut"), Suppliers.isRedAlliance, ""))
                .alongWith(launcher.stopLauncherCommand())
                .alongWith(new RunCommand(() -> indexer.stopShoot()))
                // .alongWith(intake.setToPositionCommand()) //if we need to raise the intake while driving 
        );
    }

}


