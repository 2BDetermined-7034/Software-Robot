package frc.robot.subsystems.swervedrive.photonvision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;

public class PhotonVisionReal implements PhotonSubsystem {
    private final PhotonPoseEstimator photonPoseEstimator;
    private PhotonCamera photonCamera;

    public PhotonVisionReal() {
        photonCamera = new PhotonCamera(Constants.VisionConstants.CAMERA0_NAME);
        photonPoseEstimator = new PhotonPoseEstimator(Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT,
                PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera,
                Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORM);
    }

    @Override
    public PhotonPipelineResult getPipelineResult() {
        return photonCamera.getLatestResult();
    }

    @Override
    public Optional<EstimatedRobotPose> getEstimatedPose() {
        return photonPoseEstimator.update();
    }

    @Override
    public int getBestTagID() {
        return photonCamera.getLatestResult().getBestTarget().getFiducialId();
    }

    public void update(Pose2d robotPose) {

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
        if (!photonCamera.getLatestResult().hasTargets())
            return Optional.empty();

        Transform3d transform = photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget();

        return Optional.of(new Transform2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d()));
    }

    /**
     * @param includeByID filter for tags
     * @return
     */
    @Override
    public Optional<Transform2d> getBestTagTransform(List<Integer> includeByID) {
        if (!photonCamera.getLatestResult().hasTargets())
            return Optional.empty();

        List<PhotonTrackedTarget> toteTagList = getFilteredTargetList(includeByID);

        if (toteTagList.isEmpty())
            return Optional.empty();

        // TODO: Add a second filter to 'validTransforms' to select best target.
        PhotonTrackedTarget bestTarget = toteTagList.get(0);
        Transform3d result = bestTarget.getBestCameraToTarget();
        double targetYaw = bestTarget.getYaw();
        return Optional.of(new Transform2d(result.getX(), result.getY(), result.getRotation().toRotation2d()));
    }

}
