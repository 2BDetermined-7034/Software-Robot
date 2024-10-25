package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.wpilibj.Filesystem;

import java.io.IOException;

public class AprilTagFieldLayoutLoader {
    public static AprilTagFieldLayout load(String path) {
        try {
            return new AprilTagFieldLayout(path);
        } catch (IOException e) {
            System.out.println("Failed to load field layout \"" + path + "\": " + e.getMessage());
        }
        return null;
    }
}
