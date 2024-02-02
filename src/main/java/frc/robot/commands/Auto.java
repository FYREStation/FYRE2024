package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutonomousConstants;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.DriveTrain;

/** The command for running a particular autonomous command. */
public class Auto {
    private DriveTrain dt;

    public Auto(DriveTrain dt) {
        this.dt = dt;
    }

    /**
     * Creates a new autonomous command with a Ramsete controller and
     * will return it for proper execution.
     *
     * @return - The autonomous command to run following said trajectory.
     */
    public Command getAutonomousCommand(Trajectory traj) {
        DifferentialDriveKinematics diffKinematics = 
            new DifferentialDriveKinematics(Constants.trackWidthMeters);

        SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
            AutonomousConstants.kSfeedforward, 
            AutonomousConstants.kVfeedforward,
            AutonomousConstants.kAfeedforward
        );

        DifferentialDriveVoltageConstraint voltageConstraint = 
            new DifferentialDriveVoltageConstraint(
                feedforward,
                diffKinematics,
                10
            );

        TrajectoryConfig trajConfig = new TrajectoryConfig(
            AutonomousConstants.kMaximumVelocity, 
            AutonomousConstants.kMaximumAcceleration
        ).setKinematics(diffKinematics).addConstraint(voltageConstraint);

        RamseteCommand ramsete = new RamseteCommand(
            traj, 
            dt::getPose, 
            new RamseteController(2.0, 0.7), 
            feedforward, 
            diffKinematics, 
            dt::getWheelSpeeds, 
            new PIDController(0.0035, 0.0005, 0.0001), 
            new PIDController(0.0035, 0.0005, 0.0001), 
            dt::tankDriveVolts, 
            dt
        );

        return Commands.runOnce(() -> dt.resetOdometry(traj.getInitialPose()))
            .andThen(ramsete)
            .andThen(Commands.runOnce(() -> dt.tankDriveVolts(0, 0)));
    }
}
