package frc.robot.commands.swervedrive.drivebase;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubsystemLogging;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;



/**
 * This class uses PID to drive towards a vision supplied destination pose.
 */
public class PIDToVisionPose extends Command implements SubsystemLogging {

    private final PIDController xController = new PIDController(``0.2, 0.1, 0.0);
    private final PIDController yController = new PIDController(0.2, 0.1, 0);
    private final PIDController omegaController = new PIDController(1.0, 0.0, 0.001);``

    private SwerveSubsystem drivebase;

    /**
     * Setpoint pose supplier
     */
    private final Supplier<Optional<Pose2d>> setpointSupplier;

    /**
     * Fallback cached destination pose
     */
    private Pose2d lastPose;
    /**
     * Drives to a pose supplied by {@link SwerveSubsystem#getToteDestinationPose()}
     * using PID,
     * or whatever is pointed to by {@link PIDToVisionPose#setpointSupplier}
     */
    public PIDToVisionPose(SwerveSubsystem drivebase, Supplier<Optional<Pose2d>> setpointSupplier) {
        this.drivebase = drivebase;
        addRequirements(drivebase);
        this.setpointSupplier = setpointSupplier;

        xController.setTolerance(0.075);
        yController.setTolerance(0.075);
        omegaController.setTolerance(Units.degreesToRadians(20));

        omegaController.enableContinuousInput(-Math.PI, Math.PI);

        this.lastPose = drivebase.getPose();
    }

    @Override
    public void execute() {
        var potentialSetpoint = setpointSupplier.get();
        if (potentialSetpoint.isPresent()) {
            Pose2d setpoint = potentialSetpoint.get();
            log("PID Pose Setpoint:", setpoint);
            Pose2d robotPose = drivebase.getPose();

            double vx_mps = xController.calculate(robotPose.getX(), setpoint.getX());
            double vy_mps = yController.calculate(robotPose.getY(), setpoint.getY());
            double omega_rps = omegaController.calculate(wrapRotationRadians(robotPose.getRotation()),
                    wrapRotationRadians(setpoint.getRotation()));

            drivebase.driveFieldOriented(new ChassisSpeeds(vx_mps, vy_mps, omega_rps));

            lastPose = setpoint;
        } else {
            Pose2d setpoint = lastPose;
            log("PID Pose Setpoint:", setpoint);
//            Pose2d robotPose = drivebase.getPose();

//            double vx_mps = xController.calculate(robotPose.getX(), setpoint.getX());
//            double vy_mps = yController.calculate(robotPose.getY(), setpoint.getY());
//            double omega_rps = omegaController.calculate(wrapRotationRadians(robotPose.getRotation()),
//                    wrapRotationRadians(setpoint.getRotation()));

//            drivebase.driveFieldOriented(new ChassisSpeeds(vx_mps, vy_mps, omega_rps));
        }
    }

    @Override
    public boolean isFinished() {
        return xController.atSetpoint() && yController.atSetpoint() && omegaController.atSetpoint();
    }

    @Override
    public void end(boolean interrupted) {
        drivebase.lock();
    }

    /**
     * Maps rotations from {@link Rotation2d} to the interval [-180..180]
     * Required for continuous controller input
     */
    private double wrapRotationRadians(Rotation2d rot) {
        return MathUtil.angleModulus(rot.getRadians());
    }
}
