// Vibhav: imports

package frc.robot.commands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Elevator;

/** The elevator lifting functionality for our arm. */
// Vibhav: Creates elevatorlift class and elevator var
public class ElevatorLift extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Elevator elevator;

    private TrapezoidProfile.State bottomToAmp = new TrapezoidProfile.State(1, 1);

    private TrapezoidProfile.State ampToBottom = new TrapezoidProfile.State(-1, 1);

    // Vibhav: this inits the elevator var
    public ElevatorLift(Elevator subsystem) {
        this.elevator = subsystem;
        addRequirements(subsystem);
    }


    public void execute() {
        System.out.println(elevator.getEncoderDistances());
    }

    public Command goToBottom = Commands.runOnce(() -> {
        elevator.setGoal(bottomToAmp);
    });

    public Command goToAmp = Commands.runOnce(() -> {
        elevator.setGoal(ampToBottom);
    });

    public Command enableElevator = Commands.runOnce(() -> {
        elevator.enable();
    });

    public Command runMotorForwardWhile = Commands.runOnce(() -> {
        elevator.runMotorForwardWhile();
    });

    public Command runMotorReverseWhile = Commands.runOnce(() -> {
        elevator.runMotorReverseWhile();
    });

    public Command stopMotors = Commands.runOnce(() -> {
        elevator.stopMotors();
        elevator.disable();
    });

    
}
