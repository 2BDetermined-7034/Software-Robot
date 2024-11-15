package frc.robot.commands.swervedrive.drivebase;

import java.awt.*;
import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class PathFindToTag {
    /**
     * Command to pathfind to an AprilTag on a Tote
     * TODO: decide on what to do when destination is not present
     */
    public static Command pathFindToTote(SwerveSubsystem drivebase) {
		Optional<Pose2d> toteDestinationPose = drivebase.getToteDestinationPose();
		if(toteDestinationPose.isPresent()) {
			return drivebase.driveToPose(toteDestinationPose.get()); //
		}
		return new WaitCommand(0.5);
		//Is this an auto-exclusive function?
		//We could pass in a dummy pose, and then somehow signal it in Smartdashboard / Advantage Kit
    }
}
