// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot.subsystems.swerve;

import frc.robot.Constants.MEASUREMENTS;
import frc.robot.Constants.SWERVE;
import frc.robot.Robot;
import frc.robot.helpers.SmoothingFilter;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Swerve extends SubsystemBase {
    private PIDController snapToController;

    private boolean isSlowMode;

    private SmoothingFilter smoothingFilter;
    
    private CommandSwerveDrivetrain swerve;
    
    private final SwerveRequest.FieldCentric fieldCentric = new SwerveRequest.FieldCentric()
        .withDeadband(SWERVE.MAX_SPEED * 0.1).withRotationalDeadband(SWERVE.MAX_ANGULAR_RATE * 0.01) // Add a 1% deadband
        .withDriveRequestType(DriveRequestType.Velocity); // Use closed-loop velocity control for drive motors

    private final SwerveRequest.RobotCentric robotCentric = new SwerveRequest.RobotCentric()
        .withDeadband(SWERVE.MAX_SPEED * 0.1).withRotationalDeadband(SWERVE.MAX_ANGULAR_RATE * 0.01)
        .withDriveRequestType(DriveRequestType.Velocity);

    
    public Swerve(CommandSwerveDrivetrain drivetrain) {
        swerve = drivetrain;

        smoothingFilter = new SmoothingFilter(
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.TRANSLATION_SMOOTHING_AMOUNT,
            SWERVE.ROTATION_SMOOTHING_AMOUNT
        );

        //TODO: check of the pid values are still valid
        snapToController = new PIDController(SWERVE.SNAP_TO_kP, SWERVE.SNAP_TO_kI, SWERVE.SNAP_TO_kD); // Turns the robot to a set heading
        snapToController.enableContinuousInput(-Math.PI, Math.PI); //makes -pi and pi friendly neighbors :)
        snapToController.setTolerance(Math.toRadians(2.0)); //prevent chatter and oscillation
    }

    @Override
    public void simulationPeriodic() {
        Robot.FIELD.setRobotPose(getCurrentOdometryPosition());
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

    /**
     * Send a {@code ChassisSpeeds} to the drivetrain, field-relative
     *
     * @param speeds robot-relative ChassisSpeeds speed to run the drivetrain at
     */
    public void drive(ChassisSpeeds speeds) {
        Logger.recordOutput(SWERVE.LOG_PATH+"TargetSpeeds", speeds); 

        ChassisSpeeds fieldCentricSpeeds = new ChassisSpeeds();
        fieldCentricSpeeds = fieldCentricSpeeds.fromRobotRelativeSpeeds(speeds, getYaw());

        swerve.setControl(
            fieldCentric.withVelocityX(fieldCentricSpeeds.vxMetersPerSecond)
            .withVelocityY(fieldCentricSpeeds.vyMetersPerSecond) 
            .withRotationalRate(fieldCentricSpeeds.omegaRadiansPerSecond)
        );
    }

    /**
     * Sends a robot-relative ChassisSpeeds to the drivetrain (required for PathPlanner)
     * @param speeds robot-relative ChassisSpeeds to run the drivetrain at
     */
    public void driveRobotRelative(ChassisSpeeds speeds){
        swerve.setControl(
            robotCentric.withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    //TODO: figure out how to implement the feedforwards object to drive the swerve
    //required for the pathplanner to have the feedforwards object
    public void driveRobotRelative(ChassisSpeeds speeds, DriveFeedforwards feedforwards){
        swerve.setControl(
            robotCentric.withVelocityX(speeds.vxMetersPerSecond)
            .withVelocityY(speeds.vyMetersPerSecond)
            .withRotationalRate(speeds.omegaRadiansPerSecond)
        );
    }

    /**
     * Gets the current robot-relative speed of the robot
     * 
     * @return a ChassisSpeeds speed
     */
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return swerve.getState().Speeds;
    }

    /**
     * Drives the swerve in a circle around the correct alliance's hub
     */
    public void driveInCircle(){
        double hubLocationX = 0;
        double hubLocationY = 0;

        var alliance = DriverStation.getAlliance();
        if(alliance.isPresent()){
            if(alliance.get() == DriverStation.Alliance.Red){
                hubLocationX = 4.02844;
                hubLocationY = 4.445;
            } else {
                hubLocationX = MEASUREMENTS.FIELD_X_METERS - 4.02844;
                hubLocationY = MEASUREMENTS.FIELD_Y_METERS - 4.445;
            }
        }

        double robotX = getCurrentOdometryPosition().getX();
        double robotY = getCurrentOdometryPosition().getY();

        double dx = robotX - hubLocationX;
        double dy = robotY - hubLocationY;

        double radius = Math.hypot(dx, dy);

        //add check to see if the robot is too close, modify this value later
        if(radius < 0.05)
            return;

        //this is the angular speed around the hub, so the higher this is the faster the robot will spin around the hub
        double omega = 1.0; //should probably be tuned

        //convert to field velocity
        double vxField = -dx * omega;
        double vyField = dy * omega;

        //find the rotation of the robot so the front of the robot is facing the hub
        Rotation2d angleToHub = new Rotation2d(hubLocationX - robotX, hubLocationY - robotY);

        ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(vxField, vyField, snapToAngle(angleToHub), getYaw());
        drive(speeds);

    }

    /**
     * Drives the swerve in a circle from its current position around the hub
     * @return 
     */
    public Command driveInCircleCommand(){
        return swerve.run(() -> driveInCircle());
    }

    /**
     * Define the direction the robot is facing as forward
     */
    public void resetHeading(){
        swerve.seedFieldCentric();
    }

    /**
     * Stop the swerve (feed zeros for all target velocities)
     */
    public void stop(){
        drive(new ChassisSpeeds());
    }

    /**
     * Turn all wheels into an "X" position so that the chassis effectively can't move
     */
    public SwerveRequest brake(){
        return new SwerveRequest.SwerveDriveBrake(){};
    }

    /**
     * Get the rotation of the current robot pose
     * 
     * @return Rotation2d robot rotation
     */
    public Rotation2d getYaw() {
        return swerve.getState().Pose.getRotation();
    };

    /**
     * Get the current pose of the robot
     * 
     * @return Pose2d robot pose
     */
    public Pose2d getCurrentOdometryPosition() {
        return swerve.getState().Pose;
    }

    /**
     * Sets the odometry pose of the robot to the given pose
     * 
     * @param currentPose new robot pose
     */
    public void resetPose(Pose2d currentPose) { 
        swerve.resetPose(currentPose);

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
     * 
     * @param visionRobotPoseMeters The robot pose using vision measuremnets
     * @param timestampSeconds Timestamp of the vision measurement in seconds
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        swerve.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
    }

}