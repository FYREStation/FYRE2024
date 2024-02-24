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
