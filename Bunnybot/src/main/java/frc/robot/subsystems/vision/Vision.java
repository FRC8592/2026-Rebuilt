package frc.robot.subsystems.vision;    

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CONVERSIONS;
import frc.robot.Constants.MEASUREMENTS;

public class Vision extends SubsystemBase{
    PhotonCamera camera;
    //AprilTagFieldLayout aprilTagFieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();
    PhotonPoseEstimator estimator;

    boolean targetVisible = false;
    double targetAmbiguity = 0.0;
    List<PhotonPipelineResult> results;

    VisionSystemSim visionSim;
    SimCameraProperties cameraProperties;
    PhotonCameraSim cameraSim;
    AprilTagFieldLayout aprilTagFieldLayout;

    List<AprilTag> al = new ArrayList<>();

    public Vision(String camName, Transform3d camOffsets){
        //creating new april tags for the bunny bots vision system
        al.add(new AprilTag(1, new Pose3d(72 * CONVERSIONS.INCHES_TO_METERS, 320 * CONVERSIONS.INCHES_TO_METERS, 14 * CONVERSIONS.INCHES_TO_METERS, new Rotation3d(0, 0, Math.toRadians(270)))));
        al.add(new AprilTag(2, new Pose3d(576 * CONVERSIONS.INCHES_TO_METERS, 320 * CONVERSIONS.INCHES_TO_METERS, 14 * CONVERSIONS.INCHES_TO_METERS, new Rotation3d(0, 0, Math.toRadians(270)))));
        al.add(new AprilTag(3, new Pose3d(4 * CONVERSIONS.INCHES_TO_METERS, 270 * CONVERSIONS.INCHES_TO_METERS, 14 * CONVERSIONS.INCHES_TO_METERS, new Rotation3d(0, 0, 0))));
        al.add(new AprilTag(4, new Pose3d(644 * CONVERSIONS.INCHES_TO_METERS, 270 * CONVERSIONS.INCHES_TO_METERS, 14 * CONVERSIONS.INCHES_TO_METERS, new Rotation3d(0, 0, Math.toRadians(180)))));
        al.add(new AprilTag(5, new Pose3d(4 * CONVERSIONS.INCHES_TO_METERS, 196.125 * CONVERSIONS.INCHES_TO_METERS, 46 * CONVERSIONS.INCHES_TO_METERS, new Rotation3d(0, 0, 0))));
        al.add(new AprilTag(6, new Pose3d(644 * CONVERSIONS.INCHES_TO_METERS, 196.125 * CONVERSIONS.INCHES_TO_METERS, 46 * CONVERSIONS.INCHES_TO_METERS, new Rotation3d(0, 0, Math.toRadians(180)))));
        al.add(new AprilTag(7, new Pose3d(4 * CONVERSIONS.INCHES_TO_METERS, 20.5 * CONVERSIONS.INCHES_TO_METERS, 46 * CONVERSIONS.INCHES_TO_METERS, new Rotation3d(0, 0, 0))));
        al.add(new AprilTag(8, new Pose3d(644 * CONVERSIONS.INCHES_TO_METERS, 20.5 * CONVERSIONS.INCHES_TO_METERS, 46 * CONVERSIONS.INCHES_TO_METERS, new Rotation3d(0, 0, Math.toRadians(180)))));

        aprilTagFieldLayout = new AprilTagFieldLayout(al, MEASUREMENTS.FIELD_LENGTH_METERS, MEASUREMENTS.FIELD_WIDTH_METERS);

        camera = new PhotonCamera(camName);
        estimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, camOffsets);
        visionSim = new VisionSystemSim("photonvision");

        visionSim.addAprilTags(aprilTagFieldLayout);

        cameraProperties = new SimCameraProperties();

        //TODO: redo calibration and reset values for sim
        // A 1280 x 800 camera with a 100 degree diagonal FOV.
        cameraProperties.setCalibration(640, 480, Rotation2d.fromDegrees(72.56));
        cameraProperties.setCalibError(0.39, 0.08);
        cameraProperties.setFPS(100);
        cameraProperties.setAvgLatencyMs(22);
        cameraProperties.setLatencyStdDevMs(2);

        cameraSim = new PhotonCameraSim(camera, cameraProperties);

        visionSim.addCamera(cameraSim, camOffsets);
        visionSim.getDebugField();

        cameraSim.enableRawStream(true);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);
        
    }

    @Override
    public void periodic(){
 
         // Read in relevant data from the Camera
         results = camera.getAllUnreadResults();
         if (!results.isEmpty()) {
             // Camera processed a new frame since last
             // Get the last one in the list.
             var result = results.get(results.size() - 1);
             targetVisible = result.hasTargets();
             if (targetVisible) {
                // At least one AprilTag was seen by the camera
                PhotonTrackedTarget target = result.getBestTarget();
                targetAmbiguity = target.getPoseAmbiguity();
                    
                 }
             }
             
        SmartDashboard.putBoolean("Has one tag", getTargets().size() > 0);
        SmartDashboard.putBoolean("Has two tags", getTargets().size() > 1);
    }



    /**
     * Gets the pose ambiguity ratio.
     * @return Returns the pose ambiguity ratio.
     */
    public double getPoseAmbiguityRatio(){
        return targetAmbiguity;
    }

    /**
     * Lists the targets visible by the camera.
     * @return Returns a list of the targets visible by the camera.
     */
    public List<PhotonTrackedTarget> getTargets() {
        return camera.getLatestResult().getTargets();
    }

    /**
     * Gets the current vision pose.
     * @return Returns the current vision pose.
     */
    public Optional<EstimatedRobotPose> getRobotPoseVision() {
       return estimator.update(camera.getLatestResult());
    }
}