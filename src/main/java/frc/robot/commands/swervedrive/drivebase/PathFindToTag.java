package frc.robot.commands.swervedrive.drivebase;

import java.awt.*;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PathFindToTag {

	/**
	 * Command for the drivebase to pathfind to a pose directly in front of the destination tag,
	 * with the camera facing towards the tag
	 * @param swerve drivebase Subsystem
	 * @return the path find command
	 */
	public static Command pathFindToTag(SwerveSubsystem swerve) {
		Set<Subsystem> requirements = new HashSet<>();
		requirements.add(swerve);
		return Commands.defer(
			() -> {
				SmartDashboard.putBoolean("Pathfinding running?", true);
				Optional<Pose2d> toteDestinationPose = swerve.getToteDestinationPose();
				if(toteDestinationPose.isPresent()) {
					return swerve.driveToPose(toteDestinationPose.get()).andThen(
							new InstantCommand(() -> SmartDashboard.putBoolean("Pathfinding running?", false)));
				}
				return Commands.none();
			},
			requirements
		);
	}
}
