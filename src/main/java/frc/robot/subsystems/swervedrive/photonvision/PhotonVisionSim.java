package frc.robot.subsystems.swervedrive.photonvision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class PhotonVisionSim implements PhotonSubsystem {
    private PhotonPoseEstimator photonPoseEstimator;
    private PhotonCameraSim photonCamera;
    private VisionSystemSim visionSystem;

    public PhotonVisionSim() {
        SimCameraProperties cameraProperties = new SimCameraProperties();
        cameraProperties.setCalibration(1280, 720, Rotation2d.fromDegrees(70));
        cameraProperties.setCalibError(0.25, 0.08);
        cameraProperties.setFPS(40);
        cameraProperties.setAvgLatencyMs(35);
        cameraProperties.setLatencyStdDevMs(5);

        photonCamera = new PhotonCameraSim(new PhotonCamera(Constants.VisionConstants.CAMERA0_NAME), cameraProperties);
        visionSystem = new VisionSystemSim("main");
        visionSystem.addCamera(photonCamera, Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORM);
        // visionSystem.addAprilTags(Constants.VisionConstants.BUNNY_BOTS_FIELD_LAYOUT);

        // photonPoseEstimator = new
        // PhotonPoseEstimator(Constants.VisionConstants.BUNNY_BOTS_FIELD_LAYOUT,
        // PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
        // photonCamera.getCamera(),
        // Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORM);
        photonCamera.enableRawStream(true);
        photonCamera.enableProcessedStream(true);
        photonCamera.enableDrawWireframe(true);
    }

    @Override
    public PhotonPipelineResult getPipelineResult() {
        return photonCamera.getCamera().getLatestResult();
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return photonPoseEstimator.update();
    }

    @Override
    public int getBestTagID() {
        return photonCamera.getCamera().getLatestResult().getBestTarget().getFiducialId();
    }

    /**
     * telemetry update method for the simulation robot Pose
     * 
     * @param robotPose
     */
    public void update(Pose2d robotPose) {
        visionSystem.update(robotPose);
    }

    @Override
    public List<PhotonTrackedTarget> getTargetList() {
        return getPipelineResult().getTargets();
    }

    @Override
    public List<PhotonTrackedTarget> getFilteredTargetList(List<Integer> includeByID) {
        return getTargetList().stream().filter(target -> includeByID.contains(target.getFiducialId())).toList();
    }

    @Override
    public Optional<Transform2d> getBestTagTransform() {
        if (photonCamera.getCamera().getLatestResult().hasTargets())
            return Optional.empty();

        Transform3d transform = photonCamera.getCamera().getLatestResult().getBestTarget().getBestCameraToTarget();

        return Optional.of(new Transform2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d()));
    }

    /**
     * @deprecated too different from PhotonVisionReal
     */
    @Override
    public Optional<Transform2d> getBestTagTransform(List<Integer> filterIn) {
        if (photonCamera.getCamera().getLatestResult().hasTargets())
            return Optional.empty();

        var validTransforms = photonCamera.getCamera().getLatestResult().getTargets().stream().filter((t) -> {
            for (var i : filterIn) {
                if (i == t.getFiducialId()) {
                    return false;
                }
            }

            return true;
        }).toList();

        if (validTransforms.isEmpty())
            return Optional.empty();

        Transform3d result = validTransforms.get(0).getBestCameraToTarget();
        return Optional.of(new Transform2d(result.getX(), result.getY(), result.getRotation().toRotation2d()));
    }
}
