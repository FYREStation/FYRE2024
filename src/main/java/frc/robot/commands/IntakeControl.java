package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake;

/** Acutates the intake. */
public class IntakeControl extends Command {

    // The intake subsystem
    private Intake intake;

    /**
     * Initializes a new intake controller command base.
     *
     * @param subsystem - The Intake subsystem to run off of.
     */
    public IntakeControl(Intake subsystem) {
        this.intake = subsystem;
        addRequirements(subsystem);
    }

    /**
     * Called repeatedly when a command is scheduled.
     */
    @Override
    public void execute() {
        //System.out.println(intake.getEncoderDistance());
    }
    

    /**
     * Rotates the intake at the given speed for the given ammount of seconds.

     * @param seconds - the ammount of seconds to run the intake
     * @param time - the ammount of time the intake has been running 
     * @param speed - the direction to run the intake 
     */
    public void runIntakeFor(double seconds, double time,  double speed) {
        if (time < seconds / 0.002) {
            System.out.println("Intake should be running");
            intake.runActuationDown();
            runIntakeFor(seconds, time + 1, speed);
        } else {
            intake.stopAcutation();
        }
    }

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
