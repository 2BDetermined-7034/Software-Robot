package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * common interface for SmartDashboard telemetry
 * provides shortcuts for logging various types within subsystems
 */
public interface SubsystemLogging {
    default void log(String key, String value) {
        SmartDashboard.putString(this.getClass().getName() + "/" + key, value);
    }

    default void log(String key, String[] value) {
        SmartDashboard.putStringArray(this.getClass().getName() + "/" + key, value);
    }

    default void log(String key, double value) {
        SmartDashboard.putNumber(this.getClass().getName() + "/" + key, value);
    }

    default void log(String key, double[] value) {
        SmartDashboard.putNumberArray(this.getClass().getName() + "/" + key, value);
    }

    default void log(String key, boolean value) {
        SmartDashboard.putBoolean(this.getClass().getName() + "/" + key, value);
    }

    default void log(String key, boolean[] value) {
        SmartDashboard.putBooleanArray(this.getClass().getName() + "/" + key, value);
    }

    default void log(String key, Sendable value) {
        SmartDashboard.putData(this.getClass().getName() + "/" + key, value);
    }

    default void log(String key, Transform2d value) {
        SmartDashboard.putNumberArray(this.getClass().getName() + "/" + key,
                new double[] { value.getX(), value.getY(), value.getRotation().getDegrees() });
    }

    default void log(String key, Pose2d value) {
        SmartDashboard.putNumberArray(this.getClass().getName() + "/" + key,
                new double[] { value.getX(), value.getY(), value.getRotation().getDegrees() });
    }
}
