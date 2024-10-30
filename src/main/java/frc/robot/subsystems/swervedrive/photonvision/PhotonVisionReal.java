package frc.robot.subsystems.swervedrive.photonvision;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;

import javax.swing.text.html.Option;
import java.util.ArrayList;
import java.util.Optional;
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

	public Optional<Transform2d> getBestTagOffset() {
		if (photonCamera.getLatestResult().hasTargets())
			return Optional.empty();

		Transform3d transform = photonCamera.getLatestResult().getBestTarget().getBestCameraToTarget();

		return Optional.of(new Transform2d(transform.getX(), transform.getY(), transform.getRotation().toRotation2d()));
	}

	/**
	 * @TODO Add a second filter to 'validTransforms' to select best tag.
	 * @param filterOut
	 * @return
	 */
	public Optional<Transform2d> getBestTagOffset(ArrayList<Integer> filterOut) {
		if (photonCamera.getLatestResult().hasTargets())
			return Optional.empty();

		var validTransforms = photonCamera.getLatestResult().getTargets().stream().filter((t) -> {
			for (var i: filterOut) {
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