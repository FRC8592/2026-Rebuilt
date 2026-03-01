// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.vision.Vision;
import frc.robot.Constants.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public class OdometryUpdates extends SubsystemBase {
    private Swerve swerve;
    private Vision vision;

    private Pose2d initialPose;
    private List<PhotonTrackedTarget> targets;
    private double tagCount = 0;

    /**
     * Instantiates a class that updates swerve odometry with qualifying vision poses
     * @param vision a vision subsystem that can update odometry 
     * @param swerve the swerve subsystem
     */
    public OdometryUpdates(Vision vision, Swerve swerve) {
        this.swerve = swerve;
        this.vision = vision;
    }

    @Override
    public void periodic() {
        targets = vision.getTargets();
        tagCount = targets.size();
        adjustOdometryWithVision(vision);
    }
    
    @Override
    public void simulationPeriodic() {
        Pose2d robotPose = swerve.getCurrentOdometryPosition();
        vision.simulationUpdatePose(robotPose);
    }

    /**
     * Filters vision poses and adjusts swerve odometry when the camera sees a qualifying pose
     * @param vision the vision subsystem to check for viable tags 
     */
    public void adjustOdometryWithVision(Vision vision){
        Pose2d robotPosition = new Pose2d();
        double ambiguity = -1.0;
        double timeStamp = 0.0;

        Optional<EstimatedRobotPose> robotPose = vision.getRobotPoseVision();

        if(robotPose.isEmpty())
            return;
                        
        robotPosition = robotPose.get().estimatedPose.toPose2d();
        ambiguity = vision.getPoseAmbiguityRatio();
        timeStamp = robotPose.get().timestampSeconds;
        double distanceToTag = -1.0;

        //when the vision captures apriltag(s)
        if(!targets.isEmpty()){ 
            Logger.recordOutput(VISION.LOG_PATH + "DistanceMeters", distanceToTag);
            Logger.recordOutput(VISION.LOG_PATH + "VisionPose", robotPosition);
            distanceToTag = vision.getTargets().get(0).bestCameraToTarget.getX();
        }

        // Set the initial pose for the robot before the competition starts.  It must be able to see an AprilTag
        if(DriverStation.isDisabled()) {
            initialPose = robotPosition;
            swerve.setKnownOdometryPose(initialPose);
        }

        if(DriverStation.isEnabled()) {
            // Use the multi-target pose for reliable vision poses
            if (tagCount > 1) {
                swerve.addVisionMeasurement(robotPosition, timeStamp);
            } else {
                //qualifying checks for poses derived from a single apriltag
                if ((Math.abs(ambiguity) < VISION.MAX_ACCEPTABLE_AMBIGUITY) && (tagCount > 0) 
                    && (distanceToTag < VISION.REJECT_SINGLE_TAG_POSE_ESTIMATE_RANGE)) {
                        swerve.addVisionMeasurement(robotPosition, timeStamp); 
                }
            }
            
        }
    
        Logger.recordOutput(VISION.LOG_PATH + "AmbiguityRatio", ambiguity);
        
    }
    
}