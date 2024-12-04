package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import frc.robot.commands.swervedrive.drivebase.PIDToVisionPose;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.sql.Array;
import java.util.ArrayList;

public class PathFindToLowZone extends PIDToVisionPose {
	/**
	 * Drives to a pose supplied by {@link SwerveSubsystem#getToteDestinationPose()}
	 * using PID,
	 * or whatever is pointed to by {@link PIDToVisionPose#poseSupplier}
	 *
	 * @param drivebase
	 */
	public PathFindToLowZone(SwerveSubsystem drivebase) {
		super(
			drivebase,
			() -> {
				var filter = new ArrayList<Integer>(2);
				filter.add(8);
				return drivebase.getToteDestinationPose(filter, new Transform2d(-1, 0, new Rotation2d(0)));
			}
		);
	}
}
