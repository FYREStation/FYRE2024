package frc.robot.subsystems;

import java.io.IOException;
import java.nio.file.Path;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;

public class Autonomous {
    String selectedJSON = "paths/AutonomousForward.path";
    Trajectory trajectory = new Trajectory();

    /**
     * Returns the trajectory of an autonomous path file.
     *
     * @param path - The string of the JSON file to read.
     * @return - A Trajectory mapped from the navigational JSON file.
     */
    public static Trajectory getAutonomousTrajectory(String path) {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(path);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);

            return trajectory;
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + path, ex.getStackTrace());
            return null;
        }
    }
}

