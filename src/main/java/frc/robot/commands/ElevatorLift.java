package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;

/** The elevator lifting functionality for our arm. */
public class ElevatorLift extends Command {

    // The elevator subsystem
    private Elevator elevator;

    // The top state of the elevator
    private TrapezoidProfile.State topState;

    // The bottom state of the elevator
    private TrapezoidProfile.State bottomState;

    /**
     * Creates a new elevator command.

     * @param subsystem - the elevator subsystem
     */
    public ElevatorLift(Elevator subsystem) {
        // assigns the elevator subsystem
        this.elevator = subsystem;
        // adds the elevator as a requirement
        addRequirements(subsystem);

        // assigns the top and bottom states
        topState = elevator.getDownState();
        bottomState = elevator.getUpState();
    }

    /**
     * Called repeatedly when a command is scheduled.
     */
    @Override
    public void execute() {
        //System.out.println(elevator.getEncoderDistances());
    }

    /**
     * Calibrates the elevator.
     */
    public Command calibrateLiftBounds = Commands.runOnce(() -> {
        elevator.calibrateElevatorBounds();
    });

    /**
     * Sends the elevator to top using PID.
     */
    public Command goToTop = Commands.runOnce(() -> {
        elevator.setGoal(topState);
        elevator.enable();
    });

    /**
     * Sends the elevator to the bottom using PID.
     */
    public Command goToBottom = Commands.runOnce(() -> {
        elevator.setGoal(bottomState);
        elevator.enable();
    });

    /**
     * Runs the elevator up.
     */
    public Command runMotorForward = Commands.runOnce(() -> {
        elevator.runMotorForward();
    });

    /**
     * runs the elevator down.
     */
    public Command runMotorReverse = Commands.runOnce(() -> {
        elevator.runMotorReverse();
    });

    /**
     * Stops motors using the PID stop and the motor stop.
     */
    public Command stopMotors = Commands.runOnce(() -> {
        elevator.stopMotors();
    });
}
