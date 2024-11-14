package frc.robot;

import java.io.IOException;

import edu.wpi.first.apriltag.AprilTagFieldLayout;

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
