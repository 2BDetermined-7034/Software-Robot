package frc.robot.commands.swervedrive.drivebase;

import java.util.Optional;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.SubsystemLogging;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

/**
 * This class uses PID to drive towards a vision supplied destination pose.
 */
public class PIDToVisionPose extends Command implements SubsystemLogging {

    private PIDController xController = new PIDController(2.0, 0.0, 0.0);
    private PIDController yController = new PIDController(2.0, 0.0, 0.0);
    private PIDController omegaController = new PIDController(1.0, 0.0, 0.0);

    private SwerveSubsystem drivebase;

    /**
     * Setpoint pose supplier
     */
    private Supplier<Optional<Pose2d>> setpointSupplier = () -> drivebase.getToteDestinationPose();

    /**
     * Fallback cached destination pose
     */
    private Pose2d lastPose = new Pose2d();

    /**
     * Drives to a pose supplied by {@link SwerveSubsystem.getToteDestinationPose}
     * using PID,
     * or whatever is pointed to by {@link PIDToVisionPose.poseSupplier}
     */
    public PIDToVisionPose(SwerveSubsystem drivebase) {
        this.drivebase = drivebase;
        addRequirements(drivebase);

        xController.setTolerance(0.03);
        yController.setTolerance(0.03);
        omegaController.setTolerance(3);

        omegaController.enableContinuousInput(-Math.PI, Math.PI);
    }

    @Override
    public void execute() {
        Pose2d setpoint = setpointSupplier.get().orElseGet(() -> this.lastPose);
        log("PID Pose Setpoint:", setpoint);
        Pose2d robotPose = drivebase.getPose();

        double vx_mps = xController.calculate(robotPose.getX(), setpoint.getX());
        double vy_mps = yController.calculate(robotPose.getY(), setpoint.getY());
        double omega_rps = omegaController.calculate(wrapRotationRadians(robotPose.getRotation()),
                wrapRotationRadians(setpoint.getRotation()));

        drivebase.driveFieldOriented(new ChassisSpeeds(vx_mps, vy_mps, omega_rps));

        lastPose = setpoint;

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
