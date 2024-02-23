package frc.robot.classes;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.DriveTrain;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class RamseteConveyor {
    Trajectory traj;
    Map<Integer, Command[]> interruptions;

    DriveTrain driveTrain;

    DifferentialDriveKinematics diffKinematics = 
        new DifferentialDriveKinematics(Constants.trackWidthMeters);

    SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
        AutonomousConstants.kSfeedforward, 
        AutonomousConstants.kVfeedforward,
        AutonomousConstants.kAfeedforward
    );
    
    /**
     * Initializes a new RamseteConveyor, which lets programmers create new series of Ramsete
     * controllers and add interruptions in the form of commands in between them.
     *
     * @param traj - The trajectory for the robot to follow.
     * @param interruptions - A map of intervals and commands for the robot to stop and run at.
     * @param driveTrain - The drivetrain of the robot.
     */
    public RamseteConveyor(
        Trajectory traj, 
        Map<Integer, Command[]> interruptions, 
        DriveTrain driveTrain
    ) {
        this.traj = traj;
        this.interruptions = interruptions;
        this.driveTrain = driveTrain;
    }

    /**
     * Creates an array of Trajectories which connect the ending pose of a cut trajectory
     * to the beginning pose of the following trajectory in the array.
     * 
     * <p>For example, trajectory A in the array would have an ending pose which is
     * the beginning pose of trajectory B, one index further in the trajectory array.
     * This allows the computer to use Ramsete controllers which fade into each other
     * instead of using just one Trajectory map.
     *
     * @param trajConfig - Trajectory config for the robot. Values should be
     *      built in a separate file.
     * @return - A list of trajectories which follow the rules above.
     */
    public List<Trajectory> createTrajectoryArray(TrajectoryConfig trajConfig) {
        List<Trajectory> trajList = new ArrayList<Trajectory>();
        List<State> trajStates = traj.getStates();

        for (int i = 0; i < traj.getStates().size() - 1; i++) {
            List<Pose2d> poseList = new ArrayList<Pose2d>();
            poseList.add(trajStates.get(i).poseMeters);
            poseList.add(trajStates.get(i + 1).poseMeters);

            trajList.add(TrajectoryGenerator.generateTrajectory(poseList, trajConfig));
        }

        return trajList;
    }

    /**
     * Creates a new Ramsete command, which maps to a particular trajectory
     * passed in to the method.
     *
     * @param traj - The trajectory for the Ramsete controller to follow.
     * @return - A Ramsete controller for the robot to use in autonomous.
     */
    public RamseteCommand makeRamsete(Trajectory traj) {
        RamseteCommand ramsete = new RamseteCommand(
            traj, driveTrain::getPose, // Fetches the trajectory and the initial pose of the robot.
            new RamseteController(2.0, 0.7), // Creates a ramsete controller for following.
            feedforward, diffKinematics, // Attaches the feedforward + kinematics.
            driveTrain::getWheelSpeeds, 
            new PIDController(
                AutonomousConstants.kP,
                AutonomousConstants.kI,
                AutonomousConstants.kD), // PID controller for left side.
            new PIDController(
                AutonomousConstants.kP,
                AutonomousConstants.kI,
                AutonomousConstants.kD), // PID controller for right side.
            driveTrain::tankDriveVolts, driveTrain // Sets up the drivetrain for autonomous.
        );

        return ramsete;
    }

    /**
     * Constructs a Ramsete conveyor.
     *
     * @return - The completed command, built by the Ramsete conveyor.
     */
    public Command buildConveyor() {
        Command conveyor = Commands.runOnce(() -> driveTrain.resetOdometry(traj.getInitialPose()));
        List<Trajectory> trajList = createTrajectoryArray(new TrajectoryConfig(null, null));
        
        for (int i = 0; i < trajList.size(); i++) {
            conveyor = conveyor.andThen(makeRamsete(trajList.get(i)));
            if (interruptions.containsKey(i)) {
                for (int c = 0; c < interruptions.get(i).length; i++) {
                    conveyor = conveyor.andThen(interruptions.get(i)[c]);
                }
            }
        }

        conveyor.andThen(Commands.runOnce(() -> driveTrain.tankDriveVolts(0, 0)));
        return conveyor;
    }
}
