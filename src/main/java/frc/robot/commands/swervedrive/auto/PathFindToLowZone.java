package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.swervedrive.drivebase.PIDToVisionPose;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.sql.Array;
import java.util.ArrayList;
import java.util.Optional;

public class PathFindToLowZone extends PIDToVisionPose {
	public enum Position {
		LEFT,
		MIDDLE,
		RIGHT
	}

	/**
	 * Drives to a pose supplied by {@link SwerveSubsystem#getToteDestinationPose()}
	 * using PID,
	 * or whatever is pointed to by [@TODO add something that it is pointing to]
	 *
	 * @param drivebase
	 */
	public PathFindToLowZone(SwerveSubsystem drivebase, Position position) {
		super(
			drivebase,
			() -> {
				if (DriverStation.getAlliance().isEmpty()) {
					return Optional.of(drivebase.getPose());
				}

				var alliance = DriverStation.getAlliance().get();
				var filter = new ArrayList<Integer>(1);
				if (alliance == DriverStation.Alliance.Red) {
					switch (position) {
						case LEFT:
							filter.add(14);
							break;
						case MIDDLE:
							filter.add(14);
							return drivebase.getToteDestinationPose(filter, new Transform2d(1, -2, new Rotation2d(0)));
						case RIGHT:
							filter.add(13);
							break;
					}
				} else {
					switch (position) {
						case LEFT:
							filter.add(16);
							break;
						case MIDDLE:
							filter.add(16);
							return drivebase.getToteDestinationPose(filter, new Transform2d(1, -2, new Rotation2d(0)));
						case RIGHT:
							filter.add(15);
							break;
					}
				}
				return drivebase.getToteDestinationPose(filter, new Transform2d(1, 0, new Rotation2d(0)));
			}
		);
	}
}
