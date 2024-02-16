// Vibhav: imports 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Intake;

/** Acutates the intake. */
// Vibhav: Creates intake class and intake var
public class IntakeControl extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    
    private Intake intake;

    // The current position of the elevator lift.
    // Vibhav:creates position var
    private String currentPosition = "bottom";
  
    /**
     * Initializes a new intake controller command base.
     *
     * @param subsystem - The Intake subsystem to run off of.
     */
    // Fetch the manipulator controller from the RobotContainer.
    // Vibhav: this inits the elevator var
    public IntakeControl(Intake subsystem) {
        this.intake = subsystem;
        addRequirements(subsystem);
    }

    // Vibhav: moves elevator down for intake
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
    // Vibhav: rotates intake for the amp position
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
    // Vibhav: rotates intake for the speakre position

    public Command goToSpeaker = Commands.runOnce(() -> {
        double dist = 0.0;
        if (currentPosition.equals("bottom")) {
            dist += IntakeConstants.bottomToAmpDistance;
        }

        intake.runMotorsUntil("up", IntakeConstants.ampToSpeakerDistance + dist);
        currentPosition = "speaker";
    });

    public Command intakeNote = Commands.runOnce(() -> {
        intake.spinWheels(IntakeConstants.intakeThrottle);
    });

    public Command outTakeNote = Commands.runOnce(() -> {
        intake.spinWheels(-IntakeConstants.intakeThrottle);
    });

    public Command stopIntake = Commands.runOnce(() -> {
        intake.spinWheels(0);
    });

    public Command intakeUp = Commands.runOnce(() -> {
        intake.runActuation(0.2);
    });

    public Command intakeDown = Commands.runOnce(() -> {
        intake.runActuation(-0.2);
    });

    public Command intakeStop = Commands.runOnce(() -> {
        intake.runActuation(0);
    });
}
