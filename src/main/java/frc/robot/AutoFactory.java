package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.*;

public class AutoFactory {
	SwerveSubsystem swerve;
	public Command pathToSpecificTag(int tagID) {
		Set<Subsystem> requirements = new HashSet<>();
		requirements.add(swerve);
		List<Integer> tagFilter = new ArrayList<Integer>(1);
		tagFilter.add(8);
		return Commands.defer(
				() -> {
					SmartDashboard.putBoolean("Pathfinding running?", true);
					var tags = swerve.getPhotonVision().getFilteredTargetList(tagFilter).get(0);
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
