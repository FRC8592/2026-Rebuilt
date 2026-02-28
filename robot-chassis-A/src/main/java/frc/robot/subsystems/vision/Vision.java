package frc.robot.subsystems.vision;    

import java.util.List;
import java.util.Optional;

import org.photonvision.*;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    PhotonCamera camera;
    PhotonPoseEstimator estimator;

    boolean targetVisible = false;
    double targetAmbiguity = 0.0;
    List<PhotonPipelineResult> results;

    VisionSystemSim visionSim;
    SimCameraProperties cameraProperties;
    PhotonCameraSim cameraSim;
    AprilTagFieldLayout aprilTagFieldLayout;

    /**
     * Creates a vision subsystem for a camera on the robot
     * @param camName name of the camera
     * @param camOffsets camera position relative to the robot center
     */
    public Vision(String camName, Transform3d camOffsets){
        initializeCommon(camName, camOffsets);
    }

    private void initializeCommon(String camName, Transform3d camOffsets) {
        aprilTagFieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);

        camera = new PhotonCamera(camName);
        estimator = new PhotonPoseEstimator(aprilTagFieldLayout, camOffsets);
    }

    /**
     * Creates a vision subsystem for a camera on the robot and additionally sets up simulation. 
     * Don't use this constructor for simple or early testing. 
     * @param camName name of the camera
     * @param camOffsets camera position relative to the robot center
     * @param calibrationWidth width of the calibration (i.e. the 640 of 640x480)
     * @param calibrationHeight height of the calibration (i.e. the 480 of 840x480)
     * @param calibrationAvgErrorPx average error of the calibration in pixels
     * @param calibrationErrorStdPx std in error of the calibration in pixels
     * @param fps frames per second of the calibration (i.e. 100fps, 90fps...)
     */
    public Vision(String camName, Transform3d camOffsets, int calibrationWidth, int calibrationHeight, double calibrationAvgErrorPx, double calibrationErrorStdPx, int fps){
        initializeCommon(camName, camOffsets);

        visionSim = new VisionSystemSim("photonvision");
        visionSim.addAprilTags(aprilTagFieldLayout);

        cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(calibrationWidth, calibrationHeight, Rotation2d.fromDegrees(70));
        cameraProperties.setCalibError(calibrationAvgErrorPx, calibrationErrorStdPx);
        cameraProperties.setFPS(fps);

        cameraSim = new PhotonCameraSim(camera, cameraProperties);
        cameraSim.enableRawStream(false);
        cameraSim.enableProcessedStream(true);
        cameraSim.enableDrawWireframe(true);

        visionSim.addCamera(cameraSim, camOffsets);
    }

    @Override
    public void periodic(){
        results = camera.getAllUnreadResults(); //MUST ONLY CALL THIS METHOD ONCE PER LOOP

        if (!results.isEmpty()) {
            //Get the latest result from the camera and check if it contains an apriltag
            var result = results.get(results.size() - 1);
            targetVisible = result.hasTargets();

            if (targetVisible) { //At least one AprilTag was seen by the camera
                targetAmbiguity = result.getBestTarget().getPoseAmbiguity();     
            } else {
                targetAmbiguity = -10.0; //A large value to differentiate clearly
            }
        }
        
        SmartDashboard.putNumber("Tag Count", results.size() - 1);
       // SmartDashboard.putData("VisionSimField", visionSim.getDebugField());
    }

    /**
     * Updates the vision simulation with the current swerve odometry pose
     * @param robotPose swerve odometry pose
     */
    public void simulationUpdatePose(Pose2d robotPose){
        if (visionSim == null) {
            DriverStation.reportWarning(
                "Vision simulation update skipped because visionSim is not configured for this camera.", false);
            return;
        }

        visionSim.update(robotPose);
    }

    /**
     * Gets the pose ambiguity ratio.
     * @return Returns the pose ambiguity ratio. -10 if there are no apriltags
     */
    public double getPoseAmbiguityRatio(){
        return targetAmbiguity;
    }

    /**
     * Lists the targets visible by the camera
     * @return Returns a list of the targets visible by the camera
     */
    public List<PhotonTrackedTarget> getTargets() {
        if(results == null || results.isEmpty())
            return List.of();

        var result = results.get(results.size() - 1);
        return result.getTargets();
    }

    /**
     * Gets the current vision pose
     * @return Returns the current vision pose
     */
    public Optional<EstimatedRobotPose> getRobotPoseVision() {
        if(results == null || results.isEmpty()){
            return Optional.empty();
        }

        PhotonPipelineResult latest = results.get(results.size() - 1);
        Optional<EstimatedRobotPose> pose = estimator.estimateCoprocMultiTagPose(latest);

        if(pose.isEmpty()){
            pose = estimator.estimateLowestAmbiguityPose(latest);
        }

        return pose;
    }
}
