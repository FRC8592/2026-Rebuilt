// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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
    private int IteratorCounter;
    private double[] rawXData = new double[VISION.POSE_AVERAGER_VALUE];
    private double[] rawYData = new double[VISION.POSE_AVERAGER_VALUE];
    private double[] rawThetaData = new double[VISION.POSE_AVERAGER_VALUE];

    public OdometryUpdates(Vision vision1, Swerve swerve) {
        this.swerve = swerve;
        this.vision1 = vision1;
    }

    public void periodic() {
        runVision(vision1);
    }
    
    public void simulationPeriodic() {

    }

    public static void setVision(){
        useVision = true;
    }

    public Pose2d robotPoseAverager(Optional<EstimatedRobotPose> robotPose){
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
        //Divide by the length of the array in order to derive average value
        averageX /= VISION.POSE_AVERAGER_VALUE;
        averageY /= VISION.POSE_AVERAGER_VALUE;
        averageTheta /= VISION.POSE_AVERAGER_VALUE;
        //Pose2d requires a Rotation2d object, so we create one based on the averageTheta value
        Rotation2d averageThetaRotation = new Rotation2d(averageTheta);
        return new Pose2d(averageX, averageY, averageThetaRotation);
    }

    public void runVision(Vision vision){
        if (RobotBase.isReal()){
            Pose2d robotPosition = new Pose2d();
            double ambiguity = -1d;
            double timeStamp = 0.0;
            IteratorCounter++;
    
            Optional<EstimatedRobotPose> robotPose = vision.getRobotPoseVision();
            
            if (robotPose.isPresent()){
                Pose2d placeHolder = robotPoseAverager(robotPose);
                robotPosition = robotPose.get().estimatedPose.toPose2d();
                ambiguity = vision.getPoseAmbiguityRatio();
                timeStamp = robotPose.get().timestampSeconds;

                if ((vision.getTargets().size() == 1)){
                    Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/DistanceMeters", vision.getTargets().get(0).bestCameraToTarget.getX());
                }
                }

                if ((vision.getTargets().size() > 1) || 
                   ((Math.abs(ambiguity) < VISION.MAX_ACCEPTABLE_AMBIGUITY) && 
                    (vision.getTargets().size() > 0) && 
                    (vision.getTargets().get(0).bestCameraToTarget.getX() < VISION.REJECT_SINGLE_TAG_POSE_ESTIMATE_RANGE))) 
                        { 
                            System.out.println("This is running");
                            if (DriverStation.isDisabled() && !robotPosition.equals(new Pose2d())){
                                initialPose = robotPoseAverager(robotPose);
                                swerve.setKnownOdometryPose(initialPose);
                            } else {
                                System.out.println("else is running");
                                swerve.addVisionMeasurement(robotPosition, timeStamp);
                            }
                        }
    
            }

            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/TagsInView1", vision1.getTargets().size());
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/VisionPose1", robotPosition);
            // Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/OdometryPose", swerve.getCurrentPosition());
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/AmbiguityRatio1", ambiguity);
            Logger.recordOutput(SHARED.LOG_FOLDER+"/Navigation/InitialPose", initialPose);
            
    }
    
}