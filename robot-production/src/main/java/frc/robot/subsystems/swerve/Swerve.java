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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

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
            .withDeadband(SWERVE.MAX_SPEED * 0.01).withRotationalDeadband(SWERVE.MAX_ANGULAR_RATE * 0.01) // Add a 1% deadband
            .withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop control for drive motors

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(SWERVE.MAX_SPEED * 0.01).withRotationalDeadband(SWERVE.MAX_ANGULAR_RATE * 0.01)
        .withDriveRequestType(DriveRequestType.Velocity);

    public static ChassisSpeeds speedZero = new ChassisSpeeds();

    private RobotConfig config = null;

    public Swerve(CommandSwerveDrivetrain drivetrain) {
        smoothingFilter = new SmoothingFilter(
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.ROTATION_SMOOTHING_AMOUNT
        );

        snapToController = new PIDController(SWERVE.SNAP_TO_kP, SWERVE.SNAP_TO_kI, SWERVE.SNAP_TO_kD); // Turns the robot to a set heading
        snapToController.enableContinuousInput(-Math.PI, Math.PI); //makes -pi and pi friendly neighbors :)
        snapToController.setTolerance(Math.toRadians(2.0)); //prevent chatter and oscillation

        swerve = drivetrain;

        // PathPlanner AutoBuilder configuration below. 
        https://pathplanner.dev/pplib-build-an-auto.html
        try {
            config = RobotConfig.fromGUISettings(); 
        } catch (Exception e) {
            // Handle exception as needed
            e.printStackTrace();
            //end method to prevent NullPointerException
            return;
        }

        // Configure AutoBuilder
        AutoBuilder.configure(
            this::getCurrentOdometryPosition, // Robot pose supplier
            this::setKnownOdometryPose, // Method to reset odometry (will be called if your auto has a starting pose)
            this::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            
            (speeds, feedforwards) -> driveRobotRelative(speeds), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            //TODO: replace pid constants
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(SWERVE.PATH_FOLLOW_DRIVE_KP, SWERVE.PATH_FOLLOW_DRIVE_KI, SWERVE.PATH_FOLLOW_DRIVE_KD), // Translation PID constants
                    new PIDConstants(SWERVE.PATH_FOLLOW_STEER_KP, SWERVE.PATH_FOLLOW_STEER_KI, SWERVE.PATH_FOLLOW_STEER_KD) // Rotation PID constants
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

    /**
     * Gets robot relative speeds commanded to the robot (NOT the calculated robot relative speeds)
     * @return current ChassisSpeeds
     */
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return swerve.getState().Speeds;
    }

    /**
     * Runs the SysId Quasistatic test in the given direction for the routine specified in the parameters 
     * @param direction Direction of the Quasistatic routine
     * @return Command to run
     */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return swerve.sysIdQuasistatic(direction);
    }

    /**
     * Runs the SysId Dynamic test in the given direction for the routine specified in the parameters
     * @param direction Direction of the Dynamic routine
     * @return Command to run
     */
    public Command sysIdDynamic(SysIdRoutine.Direction direction){
        return swerve.sysIdDynamic(direction);
    }

    @Override
    public void periodic() {
        Logger.recordOutput(SWERVE.LOG_PATH+"Current Pose", getCurrentOdometryPosition());

        //TODO: do we really need to run this? 
        swerve.periodic();
    }

    @Override
    public void simulationPeriodic() {
        Robot.FIELD.setRobotPose(getCurrentOdometryPosition());
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
     * Commands the swerve to stop
     * @return a command to stop the drivetrain
     */
    public Command stopCommand(){
        return swerve.runOnce(() -> stop());
    }

    /**
     * Turn all wheels into an "X" position so that the chassis effectively can't move
     */
    public SwerveRequest brake(){
        return new SwerveRequest.SwerveDriveBrake(){};
    }

    /**
     * Get the rotation of the current robot pose
     * @return Rotation2d robot rotation
     */
    public Rotation2d getYaw() {
        return swerve.getState().Pose.getRotation();
    };

    /**
     * Get the current pose of the robot
     * @return Pose2d robot pose
     */
    public Pose2d getCurrentOdometryPosition() {
        return swerve.getState().Pose;
    }

    public void setKnownOdometryPose(Pose2d currentPose) { 
        swerve.resetPose(currentPose);

        Logger.recordOutput(
            SWERVE.LOG_PATH+"Console", (
                "X: " +
                currentPose.getX()+
                "; Y: " +
                currentPose.getY() +
                "; Rotation: " +
                currentPose.getRotation().getDegrees() +
                "°."
            )
        );
    }

    /**
     * Sets whether or not the robot runs at a slower speed
     * 
     * @param slowMode whether to slow the drivetrain
     */
    public void setSlowMode(boolean slowMode) {
        this.isSlowMode = slowMode;
    }
    
    /**
     * Use PID to snap the robot to a targeted rotational heading
     *
     * @param targetHeading the heading to snap to
     * @return the rotational velocity setpoint as a Rotation2d
     */
    public double snapToAngle(Rotation2d targetHeading) {
        double currYaw = getYaw().getRadians();

        return snapToController.calculate(currYaw, targetHeading.getRadians());
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

    /**
     * Corrects the robot odometry using vision
     * @param visionRobotPoseMeters The robot pose using vision measuremnets
     * @param timestampSeconds Timestamp of the vision measurement in seconds
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        swerve.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }
}
