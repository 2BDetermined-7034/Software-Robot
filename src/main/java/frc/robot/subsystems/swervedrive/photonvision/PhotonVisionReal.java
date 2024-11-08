package frc.robot.subsystems.swervedrive.photonvision;

import edu.wpi.first.math.geometry.*;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import javax.swing.text.html.Option;
import java.util.*;
import java.util.stream.Collectors;

public class PhotonVisionReal implements PhotonSubsystem {
	private final PhotonPoseEstimator photonPoseEstimator;
	private PhotonCamera photonCamera;

	public PhotonVisionReal() {
		photonCamera = new PhotonCamera(Constants.VisionConstants.CAMERA0_NAME);
		photonPoseEstimator = new PhotonPoseEstimator(Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORM);
	}

	public PhotonPipelineResult getPipelineResult() {
		return photonCamera.getLatestResult();
	}

	public Optional<EstimatedRobotPose> getEstimatedPose() {
		return photonPoseEstimator.update();
	}

	public int getBestTagID() {
		return photonCamera.getLatestResult().getBestTarget().getFiducialId();
	}

	public void update(Pose2d robotPose) {

	}

	public List<PhotonTrackedTarget> getTargetList() {
		return getPipelineResult().getTargets();
	}

	public List<PhotonTrackedTarget> getFilteredTargetList(List<Integer> includeByID) {
		return getTargetList().stream().filter(target -> includeByID.contains(target.getFiducialId())).toList();
	}

	public Optional<Transform2d> getBestTagTransform() {
		if (!photonCamera.getLatestResult().hasTargets()) return Optional.empty();

		Transform3d transform = photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget();

		return Optional.of(new Transform2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d()));
	}

	/**
	 * @TODO Add a second filter to 'validTransforms' to select best tag.
	 * @param filterIn
	 * @return
	 */
	public Optional<Transform2d> getBestTagTransform(List<Integer> filterIn) {
		if (!photonCamera.getLatestResult().hasTargets())
			return Optional.empty();

		List<PhotonTrackedTarget> toteTagList = getFilteredTargetList(filterIn);

		if (toteTagList.isEmpty())
			return Optional.empty();

		Transform3d result = toteTagList.get(0).getBestCameraToTarget();
		return Optional.of(new Transform2d(result.getX(), result.getY(), result.getRotation().toRotation2d()));
	}


}