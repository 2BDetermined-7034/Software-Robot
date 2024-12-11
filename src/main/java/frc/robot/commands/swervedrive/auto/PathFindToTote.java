package frc.robot.commands.swervedrive.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.commands.swervedrive.drivebase.PIDToVisionPose;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.ArrayList;
import java.util.Optional;
import java.util.function.Supplier;

public class PathFindToTote extends PIDToVisionPose {
	public enum Position {
		CLOSE,
		MIDDLE,
		FAR
	}

	/**
	 * Drives to a pose supplied by {@link SwerveSubsystem#getToteDestinationPose()}
	 * using PID,
	 * or whatever is pointed to by [@TODO add something that it is pointing to]
	 *
	 * @param drivebase
	 */
	public PathFindToTote(SwerveSubsystem drivebase, PathFindToTote.Position position) {
		super(
				drivebase,
				() -> {
					if (DriverStation.getAlliance().isEmpty()) {
						return Optional.of(drivebase.getPose());
					}

					var alliance = DriverStation.getAlliance().get();
					var filter = new ArrayList<Integer>(2);
					if (alliance == DriverStation.Alliance.Red) {
						switch (position) {
							case CLOSE:
								filter.add(1);
								filter.add(12);
								break;
							case MIDDLE:
								filter.add(2);
								filter.add(11);
								break;
							case FAR:
								filter.add(3);
								filter.add(10);
								break;
						}
					} else {
						switch (position) {
							case CLOSE:
								filter.add(6);
								filter.add(7);
								break;
							case MIDDLE:
								filter.add(5);
								filter.add(8);
								break;
							case FAR:
								filter.add(4);
								filter.add(9);
								break;
						}
					}
					return drivebase.getToteDestinationPose(filter, new Transform2d(0.4, 0, new Rotation2d(0)));
				}
		);
	}
}
