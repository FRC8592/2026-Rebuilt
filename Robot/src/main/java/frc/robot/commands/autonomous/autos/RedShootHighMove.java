package frc.robot.commands.autonomous.autos;


import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Suppliers;
import frc.robot.commands.autonomous.AutoCommand;
import frc.robot.commands.largecommands.*;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Indexer;
import frc.robot.subsystems.Intake;

// public class RedShootHighMove extends AutoCommand {
//    public RedShootHighMove(String color, Launcher launcher, Indexer indexer){
//       super(
//        new FollowPathCommand(getChoreoTrajectory("RA_RedShoot"), Suppliers.isRedAlliance, "")
//        .andThen(launcher.setLauncherCommand(
//         0.4, 0.4   // bottom %, top %
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
//        new FollowPathCommand(getChoreoTrajectory("RA_RedMoveOut"), Suppliers.isRedAlliance, "")))
//    }
// }


public class RedShootHighMove extends AutoCommand {
    public RedShootHighMove(){
        super(
            launcher.setLauncherCommand(0.44, 0.30) //USE "CLOSE"
            .andThen(new WaitUntilCommand(0.5))
            .andThen(indexer.setMotorPercentOutputCommand(4, 1)).withTimeout(1.2)

            .andThen(new FollowPathCommand(getChoreoTrajectory("RedMoveOut"), Suppliers.isRedAlliance, ""))
                .alongWith(launcher.stopLauncherCommand())
                .alongWith(indexer.stopMotorCommand(4))
                // .alongWith(intake.setToPositionCommand()) //if we need to raise the intake while driving
        );
    }
}