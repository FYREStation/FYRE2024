// Vibhav: imports 

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

/** Acutates the intake. */
// Vibhav: Creates intake class and intake var
public class IntakeControl extends Command {

    // The intake subsystem
    private Intake intake;

    // The top state of the elevator
    private TrapezoidProfile.State topState;

    // The bottom state of the elevator
    private TrapezoidProfile.State bottomState;

    /**
     * Initializes a new intake controller command base.
     *
     * @param subsystem - The Intake subsystem to run off of.
     */
    public IntakeControl(Intake subsystem) {
        // assigns the intake subsystem
        this.intake = subsystem;
        // adds the intake as a requirement
        addRequirements(subsystem);

        // assigns the top and bottom states
        topState = intake.getDownState();
        bottomState = intake.getUpState();
    }

    /**
     * Called repeatedly when a command is scheduled.
     */
    @Override
    public void execute() {
        // System.out.println(intake.getEncoderDistance());
    }

    /**
     * Sends the intake to the top.
     */
    public Command goToTop = Commands.runOnce(() -> {
        intake.setGoal(topState);
        intake.enable();
    });

    /**
     * Sends the intake to the bottom.
     */
    public Command goToBottom = Commands.runOnce(() -> {
        intake.setGoal(bottomState);
        intake.enable();
    });

    /**
     * Moves the intake up.
     */
    public Command intakeUp = Commands.runOnce(() -> {
        intake.runActuationUp();;
    });

    /**
     * Moves the intake down.
     */
    public Command intakeDown = Commands.runOnce(() -> {
        intake.runActuationDown();;
    });

    /**
     * Intakes a note.
     */
    public Command intakeNote = Commands.runOnce(() -> {
        intake.intakeNote();
    });

    /**
     * Outtakes a note.
     */
    public Command outTakeNote = Commands.runOnce(() -> {
        intake.outTakeNote();
    });

    /**
     * Stops the intake wheels.
     */
    public Command stopIntakeWheels = Commands.runOnce(() -> {
        intake.stopIntakeWheels();
    });

    /**
     * Stops the intake actuation.
     */
    public Command stopIntakeActuation = Commands.runOnce(() -> {
        intake.stopAcutation();
    });
}
