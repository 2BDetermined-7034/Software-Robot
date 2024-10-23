package frc.robot.subsystems.swervedrive.photonvision;

import frc.robot.Constants;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Pose2d;

import java.util.Optional;

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
}