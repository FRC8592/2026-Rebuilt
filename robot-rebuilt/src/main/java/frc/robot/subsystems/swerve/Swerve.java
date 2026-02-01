// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.swerve;
import frc.robot.Constants.*;
import frc.robot.Robot;
import frc.robot.helpers.SmoothingFilter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class Swerve extends SubsystemBase {

    private PIDController snapToController;

    private boolean isSlowMode;

    private SmoothingFilter smoothingFilter;
    
    private CommandSwerveDrivetrain swerve;
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDeadband(SWERVE.MAX_SPEED * 0.1).withRotationalDeadband(SWERVE.MAX_ANGULAR_RATE * 0.1) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop control for drive motors

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(SWERVE.MAX_SPEED * 0.1).withRotationalDeadband(SWERVE.MAX_ANGULAR_RATE * 0.1)
        .withDriveRequestType(DriveRequestType.Velocity);

    public static ChassisSpeeds speedZero = new ChassisSpeeds();

    private ChassisSpeeds currentSpeeds = new ChassisSpeeds();

    public Swerve(CommandSwerveDrivetrain drivetrain) {
        smoothingFilter = new SmoothingFilter(
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.ROTATION_SMOOTHING_AMOUNT
        );

        //TODO: check of the pid values are still valid
        snapToController = new PIDController(SWERVE.SNAP_TO_kP, SWERVE.SNAP_TO_kI, SWERVE.SNAP_TO_kD); // Turns the robot to a set heading

        swerve = drivetrain;

        //sample code taken from pathplanner below 
        RobotConfig config = null;
        try {
            config = RobotConfig.fromGUISettings(); 
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
        }

        // Configure AutoBuilder
        AutoBuilder.configure(
            this::getPose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(2.0, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(2.0, 0.0, 0.0) // Rotation PID constants
            ),

            config, // The robot configuration

            () -> {
                // Boolean supplier that controls when the path will be mirrored for the red alliance
                // This will flip the path being followed to the red side of the field.
                // THE ORIGIN WILL REMAIN ON THE BLUE SIDE
                var alliance = DriverStation.getAlliance();
                if (alliance.isPresent())
                    return alliance.get() == DriverStation.Alliance.Red;
                return false;
                
            },

            this // Reference to this subsystem to set requirements
        );
    }

    @Override
    public void periodic() {
        Logger.recordOutput(SWERVE.LOG_PATH+"Current Pose", getPose());

        swerve.periodic();
    }

    @Override
    public void simulationPeriodic() {
        Robot.FIELD.setRobotPose(getPose());
    }

    /**
     * Send a {@code ChassisSpeeds} to the drivetrain, field-relative
     *
     * @param speeds robot-relative ChassisSpeeds speed to run the drivetrain at
     */
    public void drive(ChassisSpeeds speeds) {
        Logger.recordOutput(SWERVE.LOG_PATH+"TargetSpeeds", speeds); 

        swerve.setControl(
            fieldCentric.withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond) 
            .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    /**
     * Sends a robot-relative ChassisSpeeds to the drivetrain
     * @param speeds robot-relative ChassisSpeeds to run the drivetrain at
     */
    public void driveRobotRelative(ChassisSpeeds speeds){
        swerve.setControl(
            robotCentric.withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond)
        );

        currentSpeeds = speeds;
    }
    /**
     * Define whatever direction the robot is facing as forward
     */
    public void resetHeading(){
        swerve.seedFieldCentric();
    }

    /**
     * Stop the swerve (feed zeros for all target velocities)
     */
    public void stop(){
        drive(speedZero);
    }

    /**
     * Turn all wheels into an "X" position so that the chassis effectively can't move
     */
    // TODO: does the robot have trouble turning back to its regular rotation after the request runs?
    public SwerveRequest brake(){
        return new SwerveRequest.SwerveDriveBrake(){};
    }

    /**
     * Get the rotation of the current robot pose
     * @return Roration2d robot rotation
     */
    public Rotation2d getYaw() {
        return swerve.getState().Pose.getRotation();
    };

    /**
     * Get the current pose of the robot
     * @return Pose2d robot pose
     */
    public Pose2d getPose() {
        return swerve.getState().Pose;
    }

    /**
     * Sets the odometry pose of the robot to the given pose
     * @param currentPose new robot pose
     */
    public void resetPose(Pose2d currentPose) { 
        swerve.resetPose(currentPose);

        Logger.recordOutput(
            SWERVE.LOG_PATH + "Console", (
                "X: " +
                currentPose.getX() +
                "; Y: " +
                currentPose.getY() +
                "; Rotation: " +
                currentPose.getRotation().getDegrees() +
                "°."
            )
        );
    }

    /**
     * Sets if the robot runs at a slower speed
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
            errorAngle -= 2 * Math.PI;
        }
        else if(errorAngle <= -Math.PI){
            errorAngle += 2 * Math.PI;
        }

        double out = snapToController.calculate(0, errorAngle);

        return out;
    }

    /**
     * Process joystick inputs for swerve control
     *
     * @param rawX the raw X input from a joystick. Should be -1 to 1 (HORIZONTAL motion)
     * @param rawY the raw Y input from a joystick. Should be -1 to 1 (FORWARD motion)
     * @param rawRot the raw rotation input from a joystick. Should be -1 to 1
     *
     * @return robot-relative ChassisSpeeds
     */
    public ChassisSpeeds processJoystickInputs(double rawX, double rawY, double rawRot){
        double driveTranslateX = (
            rawX >= 0
            ? (Math.pow(Math.abs(rawX), SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawX), SWERVE.JOYSTICK_EXPONENT))
        );
        
        double driveTranslateY = (
            rawY >= 0
            ? (Math.pow(Math.abs(rawY), SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawY), SWERVE.JOYSTICK_EXPONENT))
        );

        double driveRotate = (
            rawRot >= 0
            ? (Math.pow(Math.abs(rawRot), SWERVE.JOYSTICK_EXPONENT))
            : -(Math.pow(Math.abs(rawRot), SWERVE.JOYSTICK_EXPONENT))
        );

        if(isSlowMode){
            driveTranslateX *= SWERVE.TRANSLATE_POWER_SLOW * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND;
            driveTranslateY *= SWERVE.TRANSLATE_POWER_SLOW * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND;
            driveRotate *= SWERVE.ROTATE_POWER_SLOW * SWERVE.MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND;
        } else {
            driveTranslateX *= SWERVE.TRANSLATE_POWER_FAST * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND;
            driveTranslateY *= SWERVE.TRANSLATE_POWER_FAST * SWERVE.MAX_TRANSLATIONAL_VELOCITY_METERS_PER_SECOND;
            driveRotate *= SWERVE.ROTATE_POWER_FAST * SWERVE.MAX_ROTATIONAL_VELOCITY_RADIANS_PER_SECOND;
        }

        Logger.recordOutput(SWERVE.LOG_PATH + "TranslateY", driveTranslateY);
        Logger.recordOutput(SWERVE.LOG_PATH + "TranslateX", driveTranslateX);
        Logger.recordOutput(SWERVE.LOG_PATH + "driveRotate", driveRotate);

        //returns a robot-relative ChassisSpeeds object
        //ChassisSpeeds constructor requires: (FORWARD, HORIZONTAL, ROTATION) -> (translateY, translateX, rotate)
        return smoothingFilter.smooth(new ChassisSpeeds(driveTranslateY, driveTranslateX, driveRotate));
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return currentSpeeds;
    }

    /**
     * Corrects the robot odometry using vision
     * @param visionRobotPoseMeters The robot pose using vision measuremnets
     * @param timestampSeconds Timestamp of the vision measurement in seconds
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        swerve.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }
}
