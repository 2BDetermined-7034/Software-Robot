package frc.robot.commands.swervedrive.drivebase;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.List;
import java.util.Random;

public class PathFindToTag {
	public static Command pathFindToOffset(Pose2d pose, Transform2d offset) {
		PathConstraints constraints = new PathConstraints(Constants.MAX_SPEED, Constants.MAX_ACCELERATION, Constants.MAX_ANGULAR_VELOCITY, Constants.MAX_ANGULAR_ACCELERATION);
		return AutoBuilder.pathfindToPose(pose.plus(offset), constraints, 0.0, 0.0);
	}
}
