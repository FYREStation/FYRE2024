package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.ElevatorLiftConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Elevator;

/** The elevator lifting functionality for our arm. */
public class ElevatorLift {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final Elevator elevator;

    // The current position of the elevator lift.
    private String currentPosition = "bottom";

    // Fetch the manipulator controller from the RobotContainer.
    private CommandJoystick manipulatorControl;

    public ElevatorLift(Elevator subsystem) {
        this.elevator = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Set the driverControl variable to our XboxController.
        manipulatorControl = RobotContainer.manipulatorControl;
    }

    /** Runs the elevator motors down to the bottom position on the lift.  */
    public void goToBottom() {
        double dist = 0.0;
        if (currentPosition.equals("speaker")) {
            dist += ElevatorLiftConstants.ampToSpeakerDistance;
        }

        elevator.runMotorsUntil("down", ElevatorLiftConstants.bottomToAmpDistance + dist);
        currentPosition = "bottom";
    }

    /** Runs the elevator motors up or down to the amp position on the lift.  */
    public void goToAmp() {
        if (currentPosition.equals("speaker")) {
            elevator.runMotorsUntil("down", ElevatorLiftConstants.ampToSpeakerDistance);
        }
        
        if (currentPosition.equals("bottom")) {
            elevator.runMotorsUntil("up", ElevatorLiftConstants.bottomToAmpDistance);
        }
        currentPosition = "amp";
    }

    /** Runs the elevator motors up to the speaker position on the lift.  */
    public void goToSpeaker() {
        double dist = 0.0;
        if (currentPosition.equals("bottom")) {
            dist += ElevatorLiftConstants.bottomToAmpDistance;
        }

        elevator.runMotorsUntil("up", ElevatorLiftConstants.ampToSpeakerDistance + dist);
        currentPosition = "speaker";
    }
}
