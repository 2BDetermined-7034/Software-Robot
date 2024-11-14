package frc.robot.commands.swervedrive.drivebase;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PathFindToTag {
    /**
     * Command to pathfind to an AprilTag on a Tote
     */
    public static Command pathFindToTote(SwerveSubsystem drivebase) {
		Optional<Pose2d> toteDestinationPose = drivebase.getToteDestinationPose();
		if(toteDestinationPose.isPresent()) {
			return drivebase.driveToPose(toteDestinationPose.get());
		}
		return new WaitCommand(4);
    }
}
