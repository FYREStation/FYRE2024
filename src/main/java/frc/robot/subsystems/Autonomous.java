package frc.robot.subsystems;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.io.IOException;
import java.nio.file.Path;

/** The autonomous subsystem for controller-less movement. */
public class Autonomous extends SubsystemBase {
    String trajectoryJson;
    Trajectory trajectory = new Trajectory();

    /**
     * Creates a new autonomous subsytem based on the input
     * JSON path, which is used for autonomous movement.
     *
     * @param trajectoryJson - The String path for the JSON file.
     */
    public Autonomous(String trajectoryJson) {
        this.trajectoryJson = trajectoryJson;
    }

    /**
     * Returns the trajectory of an autonomous path file.
     *
     * @return - A Trajectory mapped from the navigational JSON file.
     */
    public Trajectory getAutonomousTrajectory() {
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJson);
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
            return trajectory;
        } catch (IOException ex) {
            DriverStation.reportError(
                "Unable to open trajectory: " + trajectoryJson, 
                ex.getStackTrace()
            );

            return null;
        }
    }

    

}

