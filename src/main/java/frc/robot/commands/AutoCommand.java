package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.DriveTrain;

/** The command for running a particular autonomous trajectory. */
public class AutoCommand extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    // auto needs to be added as a requirement or the runtime gets mad.
    private final Autonomous auto;
    private final DriveTrain driveTrain;

    /**
     * Initializes a new autonomous command to drive based on the 
     * drivetrain subsystem passed in.
     *
     * @param driveTrain - The drivetrain subsystem of the robot
     */
    public AutoCommand(Autonomous auto, DriveTrain driveTrain) {
        this.auto = auto;
        this.driveTrain = driveTrain;
        addRequirements(auto);
    }

    /**
     * Creates a new autonomous command with a Ramsete controller and
     * will return it for proper execution.
     *
     * @return - The autonomous command to run following said trajectory.
     */
    public Command getAutonomousCommand(Trajectory traj) {
        // Creates a new differential drive kinematics scematic.
        DifferentialDriveKinematics diffKinematics = 
            new DifferentialDriveKinematics(Constants.trackWidthMeters);

        // Creates a new feedforward based on robot-specific constants.
        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            AutonomousConstants.kSfeedforward, 
            AutonomousConstants.kVfeedforward,
            AutonomousConstants.kAfeedforward
        );

        // Creates a new Ramsete Command to run the autonomous PathWeaver code.
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

        // Resets robot positioning, runs the autonomous command, and then stops the robot.
        return Commands.runOnce(() -> driveTrain.resetOdometry(traj.getInitialPose()))
            .andThen(ramsete)
            .andThen(Commands.runOnce(() -> driveTrain.tankDriveVolts(0, 0)));
    }
}
