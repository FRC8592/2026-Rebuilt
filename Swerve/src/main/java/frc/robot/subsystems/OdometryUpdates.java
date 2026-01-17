// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.lang.reflect.Array;
import java.util.ArrayList;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;

import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.*;


public class OdometryUpdates extends SubsystemBase {

    private Swerve swerve;
    private Vision vision1;
    private Pose2d initialPose;
    private static boolean useVision;
    private double ambiguity;
    private Pose2d robotPosition;
    private int IteratorCounter = 0;
    private double[] rawXData = new double[VISION.POSE_AVERAGER_VALUE];
    private double[] rawYData = new double[VISION.POSE_AVERAGER_VALUE];
    private double[] rawThetaData = new double[VISION.POSE_AVERAGER_VALUE];

    public OdometryUpdates(Vision vision1, Swerve swerve) {
        this.swerve = swerve;
        this.vision1 = vision1;
    }

    @Override
    public void periodic() {
        runVision(vision1);
    }
    
    public void simulationPeriodic() {

    }

    public static void setVision(){
        useVision = true;
    }

    public Pose2d robotPoseAverager(Optional<EstimatedRobotPose> robotPose) {
        /*
         * Derive a new Pose2d object from the robotPose
         * We refer to this as rawRobotPosition as this is the raw unaveraged data derived from camera data
         */
        Pose2d rawRobotPosition = robotPose.get().estimatedPose.toPose2d();
        /*
         *  Update each array with current data from the robot.
         *  This overwrites the old data, and contributes to our averaging logic.
         */
        rawXData[IteratorCounter % VISION.POSE_AVERAGER_VALUE] = rawRobotPosition.getX();
        rawYData[IteratorCounter % VISION.POSE_AVERAGER_VALUE] = rawRobotPosition.getY();
        rawThetaData[IteratorCounter % VISION.POSE_AVERAGER_VALUE] = rawRobotPosition.getRotation().getRadians();
        //To ensure we are keeping track of iterations
        IteratorCounter++;
        //These variables are initialized to derive averages from their respective arrays
        double averageX = 0.0;
        double averageY = 0.0;
        double averageTheta = 0.0;
        //We traverse throughout out all arrays, summing up the data from them into the average variables
        for(int i = 0; i< VISION.POSE_AVERAGER_VALUE; i++){
            averageX += rawXData[i];
            averageY += rawYData[i];
            averageTheta += rawThetaData[i];
        }

        //logic is incorrect because angular space is circular, wraps at -180 to 0
        // //Divide by the length of the array in order to derive average value
        averageX /= VISION.POSE_AVERAGER_VALUE;
        averageY /= VISION.POSE_AVERAGER_VALUE;
        averageTheta /= VISION.POSE_AVERAGER_VALUE;
        // //Pose2d requires a Rotation2d object, so we create one based on the averageTheta value
        // Rotation2d averageThetaRotation = new Rotation2d(averageTheta);

        double sumSin = 0;
        double sumCos = 0;
        for (double a : rawThetaData) {
            sumSin += Math.sin(a);
            sumCos += Math.cos(a);
        }
        double avgTheta = Math.atan2(sumSin, sumCos);

        return new Pose2d(averageX, averageY, new Rotation2d(avgTheta));
        
    }

    public void runVision(Vision vision) {
        if (RobotBase.isReal()) {
            Pose2d robotPosition = new Pose2d();
            double ambiguity = -1d;
            double timeStamp = 0.0;
            IteratorCounter++;
    
            Optional<EstimatedRobotPose> robotPose = vision.getRobotPoseVision();
            
            if (robotPose.isPresent()) {
                Pose2d placeHolder = robotPoseAverager(robotPose);
                robotPosition = robotPose.get().estimatedPose.toPose2d();
                ambiguity = vision.getPoseAmbiguityRatio();
                timeStamp = robotPose.get().timestampSeconds;

                if ((vision.getTargets().size() >= 1)){
                    Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/DistanceMeters", vision.getTargets().get(0).bestCameraToTarget.getX());
                    Logger.recordOutput(SHARED.LOG_FOLDER+"/Vision/VisionPose", robotPosition);
                }

                // Set the initial pose for the robot before the competition starts.  It must be able to see an AprilTag
                if (DriverStation.isDisabled() && !robotPosition.equals(new Pose2d())) {
                    initialPose = robotPoseAverager(robotPose);
                    swerve.setKnownOdometryPose(initialPose);
                }

                if (!DriverStation.isDisabled() && !robotPosition.equals(new Pose2d())) {
                    // If we have more than one target in view, we always seem to get reliable vision poses.
                    // Use the multi-target pose without further qualification
                    if (vision.getTargets().size() > 1) {
                        swerve.addVisionMeasurement(robotPosition, timeStamp);
                    }

                    // If we don't have a multi-target pose, we need to add qualifying checks to ensure we have a good pose
                    else {
                        if ((Math.abs(ambiguity) < VISION.MAX_ACCEPTABLE_AMBIGUITY) && 
                            (vision.getTargets().size() > 0) && 
                            (vision.getTargets().get(0).bestCameraToTarget.getX() < VISION.REJECT_SINGLE_TAG_POSE_ESTIMATE_RANGE)) {
                                swerve.addVisionMeasurement(robotPosition, timeStamp); 
                        }
                    }
                }
            }

            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/TagsInView1", vision1.getTargets().size());
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/VisionPose1", robotPosition);
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/OdometryPose", swerve.getCurrentOdometryPosition());
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/AmbiguityRatio1", ambiguity);
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/InitialPose", initialPose);
        }
    }
    
}