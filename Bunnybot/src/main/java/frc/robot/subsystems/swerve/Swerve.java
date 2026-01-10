// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.swerve;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants.*;
import frc.robot.Robot;
import frc.robot.helpers.SmoothingFilter;

public class Swerve extends SubsystemBase {

    private PIDController snapToController;

    private boolean isSlowMode;

    private SmoothingFilter smoothingFilter;
    
    private CommandSwerveDrivetrain swerve;
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
            .withDeadband(SWERVE.MAX_SPEED * 0.1).withRotationalDeadband(SWERVE.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use open-loop control for drive motors
    public static ChassisSpeeds speedZero = new ChassisSpeeds();

    public Swerve(CommandSwerveDrivetrain drivetrain) {
        smoothingFilter = new SmoothingFilter(
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.ROTATION_SMOOTHING_AMOUNT
        );

        snapToController = new PIDController(SWERVE.SNAP_TO_kP, SWERVE.SNAP_TO_kI, SWERVE.SNAP_TO_kD); // Turns the robot to a set heading

        // TODO: Any initialization code needed for the new swerve stuff
        swerve = drivetrain;
    }

    @Override
    public void periodic() {
        // TODO: Periodic logging
        Logger.recordOutput(SWERVE.LOG_PATH+"Current Pose", getCurrentOdometryPosition());
        swerve.periodic();
    }

    public void simulationPeriodic() {
        Pose2d pose = new Pose2d(
            getCurrentOdometryPosition().getTranslation(),
            getCurrentOdometryPosition().getRotation()
        );

        Robot.FIELD.setRobotPose(pose==null ? new Pose2d() : pose);
    }

    /**
     * Stop the swerve (feed zeros for all target velocities)
     */
    public void stop(){
        drive(speedZero);
    }

    /**
     * Send a {@code ChassisSpeeds} to the drivetrain, robot-relative
     *
     * @param speeds the speeds to run the drivetrain at
     */
    public void drive(ChassisSpeeds speeds) {
        Logger.recordOutput(SWERVE.LOG_PATH+"TargetSpeeds", speeds); 

        swerve.setControl(
            fieldCentric.withVelocityX(speeds.vxMetersPerSecond) // Drive forward with negative Y (forward)
            .withVelocityY(speeds.vyMetersPerSecond) // Drive left with negative X (left)
            .withRotationalRate(speeds.omegaRadiansPerSecond));
    }


    /**
     * Define whatever direction the robot is facing as forward
     */
    public void resetHeading(){
        System.out.println("Running reset heading");
        // TODO: implement something that allows the commented code to work
        swerve.seedFieldCentric();
    }

    public Rotation2d getYaw() {
        return swerve.getState().Pose.getRotation();
    };

    public Pose2d getCurrentOdometryPosition() {
        return swerve.getState().Pose;
    }

    public void setKnownOdometryPose(Pose2d currentPose) {        
        swerve.resetPose(currentPose);
        Logger.recordOutput(
            SWERVE.LOG_PATH+"Console", (
                "Current pose reset to X: "+
                currentPose.getX()+
                "; Y: "+
                currentPose.getY()+
                "; Rotation: "+
                -currentPose.getRotation().getDegrees()+
                "°."
            )
        );
    }
    
    public void resetPose(Pose2d pose, boolean flip) {
        // TODO: implement something that allows the commented code to work
        if(flip){
            Pose2d flipped = new Pose2d(
                new Translation2d(
                    MEASUREMENTS.FIELD_LENGTH_METERS-pose.getX(),
                    pose.getY()
                ),
                Rotation2d.fromDegrees(180).minus(pose.getRotation())
            );
            setKnownOdometryPose(flipped);
            return;
        }
        setKnownOdometryPose(pose);
    }

    /**
     * Sets whether or not the input joystick is slowed
     * 
     * @param slowMode whether to slow the drivetrain
     */
    public void setSlowMode(boolean slowMode) {
        this.isSlowMode = slowMode;
    }
    /**
     * Use PID to snap the robot to a rotational setpoint
     *
     * @param setpoint the setpoint to snap to
     * @return the rotational velocity setpoint as a Rotation2d
     */
    public double snapToAngle(Rotation2d setpoint) {
        double currYaw = Math.toRadians(getYaw().getDegrees()%360);
        double errorAngle = setpoint.getRadians() - currYaw;

        if(errorAngle > Math.PI){
            errorAngle -= 2*Math.PI;
        }
        else if(errorAngle <= -Math.PI){
            errorAngle += 2*Math.PI;
        }

        double out = snapToController.calculate(0, errorAngle);

        return out;
    }

    /**
     * Process joystick inputs for human control
     *
     * @param rawX the raw X input from a joystick. Should be -1 to 1
     * @param rawY the raw Y input from a joystick. Should be -1 to 1
     * @param rawRot the raw rotation input from a joystick. Should be -1 to 1
     * @param fieldRelativeAllowed if this is true, switch between field- and
     * robot-relative based on {@link Swerve#robotRelative}. Otherwise, force
     * robot-relative.
     *
     * @return a ChassisSpeeds ready to be sent to the swerve.
     */
    public ChassisSpeeds processJoystickInputs(double rawX, double rawY, double rawRot){
        double driveTranslateY = (
            rawY >= 0
            ? (Math.pow(Math.abs(rawY), SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawY), SWERVE.JOYSTICK_EXPONENT))
        );

        double driveTranslateX = (
            rawX >= 0
            ? (Math.pow(Math.abs(rawX), SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawX), SWERVE.JOYSTICK_EXPONENT))
        );

        double driveRotate = (
            rawRot >= 0
            ? (Math.pow(Math.abs(rawRot), SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawRot), SWERVE.JOYSTICK_EXPONENT))
        );

        Logger.recordOutput(SWERVE.LOG_PATH+"TranslateY", driveTranslateY);
        Logger.recordOutput(SWERVE.LOG_PATH+"TranslateX", driveTranslateX);
        Logger.recordOutput(SWERVE.LOG_PATH+"driveRotate", driveRotate);

        ChassisSpeeds currentSpeeds;

        if (isSlowMode) {
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                driveTranslateY * SWERVE.TRANSLATE_POWER_SLOW * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND * 0.5,
                driveTranslateX * SWERVE.TRANSLATE_POWER_SLOW * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND * 0.5,
                driveRotate * SWERVE.ROTATE_POWER_SLOW * SWERVE.MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND * 0.5
            ));
        }
        else {
            currentSpeeds = smoothingFilter.smooth(new ChassisSpeeds(
                driveTranslateY * SWERVE.TRANSLATE_POWER_FAST * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveTranslateX * SWERVE.TRANSLATE_POWER_FAST * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND,
                driveRotate * SWERVE.ROTATE_POWER_FAST * SWERVE.MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND
            ));
        }

        return currentSpeeds;
    }

    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        swerve.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }
}
