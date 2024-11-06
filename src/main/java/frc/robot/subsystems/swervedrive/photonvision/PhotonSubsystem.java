package frc.robot.subsystems.swervedrive.photonvision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import frc.robot.Constants;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;

import java.util.ArrayList;
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

	public Optional<Transform2d> getBestTagOffset();
	public Optional<Transform2d> getBestTagOffset(List<Integer> filterIn);
}
