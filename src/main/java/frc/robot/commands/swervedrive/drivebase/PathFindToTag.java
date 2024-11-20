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
	public static Command pathFindToTag(SwerveSubsystem swerve) {
		Set<Subsystem> requirements = new HashSet<Subsystem>();
		requirements.add(swerve);
		return Commands.defer(
			() -> {
				Optional<Pose2d> toteDestinationPose = swerve.getToteDestinationPose();
				if(toteDestinationPose.isPresent()) {
					return swerve.driveToPose(toteDestinationPose.get());
				}
				return Commands.none();
			},
			requirements
		);
	}
}
