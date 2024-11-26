package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import frc.robot.Constants.PathFindToTagPIDConstants;
import java.awt.*;
import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

public class PathFindToTagPID extends Command {
    //TODO: Create PID constants
    private SwerveSubsystem swerve;
    private PIDController pidX = new PIDController(1.0,0.0,0.0);
    private PIDController pidY = new PIDController(1.0,0.0,0.0);
    private PIDController pidRot =  new PIDController(1.0, 0.0, 0.0);

    public PathFindToTagPID(SwerveSubsystem swerve) {
        addRequirements(swerve);
        this.swerve = swerve;
        Optional<Pose2d> destinationPose = swerve.getToteDestinationPose();

        if (destinationPose.isPresent()) {
            pidX.setSetpoint(destinationPose.get().getX());
            pidY.setSetpoint(destinationPose.get().getY());
            pidRot.setSetpoint(destinationPose.get().getRotation().getRadians());
            pidX.setTolerance(PathFindToTagPIDConstants.PIDX_TOLERANCE);
            pidY.setTolerance(PathFindToTagPIDConstants.PIDY_TOLERANCE);
            pidRot.setTolerance(PathFindToTagPIDConstants.PID_ROT_TOLERANCE);
            pidRot.enableContinuousInput(0, 2*Math.PI);
        }
    }

    @Override
    public void execute() {
        if (swerve.getToteDestinationPose().isPresent()){
            swerve.drive(new Translation2d(pidX.calculate(swerve.getPose().getX()),pidY.calculate(swerve.getPose().getY())),
                    pidRot.calculate(swerve.getPose().getRotation().getRotations()), true);
        }
    }

    @Override
    public boolean isFinished() {
        return pidX.atSetpoint() && pidY.atSetpoint() && pidRot.atSetpoint();
    }
}