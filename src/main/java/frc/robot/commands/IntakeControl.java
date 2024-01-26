package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import frc.robot.Constants.IntakeConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Intake;

/** Acutates the intake. */
public class IntakeControl extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Intake intake;

    // The current position of the elevator lift.
    private String currentPosition = "bottom";

    // Fetch the manipulator controller from the RobotContainer.
    private CommandJoystick manipulatorControl;

    public IntakeControl(Intake subsystem) {
        this.intake = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Set the driverControl variable to our XboxController.
        manipulatorControl = RobotContainer.manipulatorControl;
    }

    /** Runs the elevator motors down to the bottom position on the lift.  */
    public Command goToBottom = Commands.runOnce(() -> {
        double dist = 0.0;
        if (currentPosition.equals("speaker")) {
            dist += IntakeConstants.ampToSpeakerDistance;
        }

        intake.runMotorsUntil("down", IntakeConstants.bottomToAmpDistance + dist);
        currentPosition = "bottom";
    });

    /** Runs the elevator motors up or down to the amp position on the lift.  */
    public Command goToAmp = Commands.runOnce(() -> {
        if (currentPosition.equals("speaker")) {
            intake.runMotorsUntil("down", IntakeConstants.ampToSpeakerDistance);
        }
        
        if (currentPosition.equals("bottom")) {
            intake.runMotorsUntil("up", IntakeConstants.bottomToAmpDistance);
        }
        currentPosition = "amp";
    });

    /** Runs the elevator motors up to the speaker position on the lift.  */
    public Command goToSpeaker = Commands.runOnce(() -> {
        double dist = 0.0;
        if (currentPosition.equals("bottom")) {
            dist += IntakeConstants.bottomToAmpDistance;
        }

        intake.runMotorsUntil("up", IntakeConstants.ampToSpeakerDistance + dist);
        currentPosition = "speaker";
    });

    public Command intakeNote = Commands.run(() -> {
        intake.spinWheels(IntakeConstants.intakeThrottle);
    });

    public Command outTakeNote = Commands.run(() -> {
        intake.spinWheels(-IntakeConstants.intakeThrottle);
    });

    public Command stopIntake = Commands.runOnce(() -> {
        intake.spinWheels(0);
    });
}
