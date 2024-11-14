package frc.robot.subsystems.swervedrive.photonvision;

import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;

/**
 * interface for our custom photonvision methods
 */
public interface PhotonSubsystem {
    public PhotonPipelineResult getPipelineResult();

    public Optional<EstimatedRobotPose> getEstimatedPose();

    public int getBestTagID();

    public void update(Pose2d robotPose);

    /**
     * @return list of visible vision targets
     */
    public List<PhotonTrackedTarget> getTargetList();

    /**
     * @return list of vision vision targets with an inclusive filter
     */
    public List<PhotonTrackedTarget> getFilteredTargetList(List<Integer> includeByID);

    /**
     * @return Transform2d to the least ambiguous apriltag target
     */
    public Optional<Transform2d> getBestTagTransform();

    /**
     * @return Transform2d to the least ambiguous apriltag target wtihin the filter
     */
    public Optional<Transform2d> getBestTagTransform(List<Integer> filterIn);
}
