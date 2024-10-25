package frc.robot;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;

import java.util.ArrayList;
import java.util.List;

public class TagLoader {
	public static AprilTagFieldLayout bunnyBots2024() {
		List<AprilTag> aprilTags = new ArrayList<AprilTag>(1);
		aprilTags.add(new AprilTag(
				14,
				new Pose3d(new Translation3d(0, 2.286, 2.64583333333), new Rotation3d())
		));

		AprilTagFieldLayout result = new AprilTagFieldLayout(aprilTags, 16.459254,8.2296);

		return result;
	}
}
