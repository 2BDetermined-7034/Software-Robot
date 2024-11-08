package frc.robot.subsystems.swervedrive.photonvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonTrackedTarget;

public interface PhotonSubsystem {
	public PhotonPipelineResult getPipelineResult();
    
	public Optional<EstimatedRobotPose> getEstimatedPose();

	public int getBestTagID();

	public void update(Pose2d robotPose);
	public List<PhotonTrackedTarget> getTargetList();
	public List<PhotonTrackedTarget> getFilteredTargetList(List<Integer> includeByID);

	public Optional<Transform2d> getBestTagTransform();
	public Optional<Transform2d> getBestTagTransform(List<Integer> filterIn);
}
