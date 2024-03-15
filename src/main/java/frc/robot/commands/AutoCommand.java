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
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

/** The command for running a particular autonomous trajectory. */
public class AutoCommand extends Command {

    // auto needs to be added as a requirement or the runtime gets mad.
    private final Autonomous auto;
    private final DriveTrain driveTrain;
    private final Intake intake;
    private final Elevator elevator;

    /**
     * Initializes a new autonomous command to drive based on the 
     * drivetrain subsystem passed in.
     *
     * @param driveTrain - The drivetrain subsystem of the robot
     */
    public AutoCommand(Autonomous auto, DriveTrain driveTrain, Intake intake, Elevator elevator) {
        this.auto = auto;
        this.driveTrain = driveTrain;
        this.intake = intake;
        this.elevator = elevator;
        addRequirements(auto);
    }

    /**
     * Creates a new autonomous command with a Ramsete controller and
     * will return it for proper execution.
     *
     * @return - The autonomous command to run following said trajectory.
     */
    public Command getAutonomousCommand() {
        System.out.println("fetched");

        // Fetches the trajectory from the autonomous subsystem.
        Trajectory traj = auto.getAutonomousTrajectory();

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
            new RamseteController(0.7, 2.0), // Creates a ramsete controller for following.
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
            driveTrain::tankDriveVolts // Sets up the drivetrain for autonomous.
        );

        System.out.println("ramsete");

        // Resets robot positioning, runs the autonomous command, and then stops the robot.
        return Commands.runOnce(() -> {
            System.out.println("reset");
            driveTrain.resetOdometry(traj.getInitialPose());
        })
            .andThen(ramsete).andThen(Commands.runOnce(() ->
                System.out.println("completed ramsete")))
            .andThen(Commands.runOnce(() -> driveTrain.tankDriveVolts(0, 0)));
    }

    /**
     * Runs an autonomous without pid control for backups.

     * @return command - the command to run
     */
    public Command getAutoNoPid() {
        return Commands.runOnce(() -> {
            elevator.goToTop();;
            while (driveTrain.getEncoderPositions()[0] < 12) {
                driveTrain.arcadeDrive(0.2, 0);
                continue;
            }
            driveTrain.arcadeDrive(0, 0);
            while (intake.runIntakeFor(2, -0.1)) {
                continue;
            }
            intake.outTakeNote();
        });
    }
}
