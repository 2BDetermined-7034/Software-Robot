package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

import java.util.HashSet;
import java.util.Optional;
import java.util.Set;

public class PathFindToTagPID {
    //TODO: Create PID constants + figure out tolerances
    private static PIDController pidX = new PIDController(0.0,0.0,0.0);
    private static PIDController pidY = new PIDController(0.0,0.0,0.0);
    private static PIDController pidRot =  new PIDController(0.0, 0.0, 0.0);

    public static void PathFindToTag (SwerveSubsystem swerve, double[] tolerances) {
        Set<Subsystem> requirements = new HashSet<>();
        requirements.add(swerve);
        Optional<Pose2d> destinationPose = Optional.of(swerve.getToteDestinationPose().get());
        if(destinationPose.isPresent()) {
            pidX.setSetpoint(destinationPose.get().getX());
            pidY.setSetpoint(destinationPose.get().getY());
            pidRot.setSetpoint(destinationPose.get().getRotation().getDegrees());
            pidX.setTolerance(tolerances[0]);
            pidY.setTolerance(tolerances[1]);
            pidRot.setTolerance(tolerances[2]);
            while(!pidX.atSetpoint() && !pidY.atSetpoint() && !pidRot.atSetpoint()) {
                swerve.driveToPose(new Pose2d(pidX.calculate(swerve.getPose().getX()), pidY.calculate(swerve.getPose().getY()),
                        Rotation2d.fromDegrees(pidRot.calculate(swerve.getPose().getRotation().getDegrees()))));

            }
        }
    }
}