package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;

public final class NewtonCommands {  
    
    public static void addSubsystems(){
        
       
    }

    /**
     * Command to drive the swerve with translation processed for human input and
     * rotation controlled by the snap-to PID controller (snapping to the passed-in)
     * angle
     *
     * @param angle the angle to snap to
     * @param driveX a lambda returning the driver's X input
     * @param driveY a lambda returning the driver's Y input
     *
     * @return the command
     */
    // public static Command swerveSnapToCommand(Rotation2d angle, DoubleSupplier driveX, DoubleSupplier driveY){
    //     return swerve.run(() -> {
    //         ChassisSpeeds processed = swerve.processJoystickInputs(
    //             driveX.getAsDouble(),
    //             driveY.getAsDouble(),
    //             0
    //         );
    //         processed.omegaRadiansPerSecond = swerve.snapToAngle(angle);
    //         swerve.drive(processed, DriveModes.AUTOMATIC);
    //     });
    // }
}

