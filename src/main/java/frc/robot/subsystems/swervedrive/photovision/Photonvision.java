package frc.robot.subsystems.swervedrive.photovision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;

public class Photonvision {
	private final PhotonCamera photonCamera;
	private Pose2d previous;
	private final PhotonPoseEstimator photonPoseEstimator;
		public Photonvision(){
		photonCamera = new PhotonCamera(Constants.VisionConstants.CAMERA1_NAME);
		photonPoseEstimator = new PhotonPoseEstimator(Constants.VisionConstants.APRIL_TAG_FIELD_LAYOUT, PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, photonCamera, Constants.VisionConstants.ROBOT_TO_CAMERA_TRANSFORM);
	}

	public PhotonPipelineResult getPipelineResult(){
		return photonCamera.getLatestResult();
	}

	public boolean hasTargets(){
		return photonCamera.hasTargets();
	}

	public Pose2d getEstimatedPose() {
		var estimated = photonPoseEstimator.update();
		if (estimated.isPresent()) {
			previous = estimated.get().estimatedPose.toPose2d();
		}

		return previous;
	}

//	public Pose2d getRelativePose(){
//
//	}
//
//	public Pose2d getFieldRelative(){
//	}
}
