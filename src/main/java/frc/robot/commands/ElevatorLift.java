// Vibhav: imports

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.ElevatorLiftConstants;
import frc.robot.subsystems.Elevator;

/** The elevator lifting functionality for our arm. */
// Vibhav: Creates elevatorlift class and elevator var
public class ElevatorLift extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private Elevator elevator;

    // The current position of the elevator lift.
    // Vibhav:creates position var
    private String currentPosition = "bottom";

    // Vibhav: this inits the elevator var
    public ElevatorLift(Elevator subsystem) {
        this.elevator = subsystem;
        addRequirements(subsystem);
    }


    public void execute() {
        System.out.println(elevator.getEncoderDistances());
    }


    /** Runs the elevator motors down to the bottom position on the lift.  */
    // Vibhav: makes the elevator go to bottom
    public Command goToBottom = Commands.runOnce(() -> {
        double dist = 0.0;
        if (currentPosition.equals("speaker")) {
            dist += ElevatorLiftConstants.ampToSpeakerDistance;
        }

        elevator.runMotorsUntil("down", ElevatorLiftConstants.bottomToAmpDistance + dist);
        currentPosition = "bottom";
    });

    // /** Runs the elevator motors up or down to the amp position on the lift.  */
    // Vibhav: if at bottom, go up, if at top go down (go to amp) 
    public Command goToAmp = Commands.runOnce(() -> {
        if (currentPosition.equals("speaker")) {
            elevator.runMotorsUntil("down", ElevatorLiftConstants.ampToSpeakerDistance);
        }

        if (currentPosition.equals("bottom")) {
            elevator.runMotorsUntil("up", ElevatorLiftConstants.bottomToAmpDistance);
        }
        currentPosition = "amp";
    });

    // /** Runs the elevator motors up to the speaker position on the lift.  */
    // Vibhav: 
    public Command goToSpeaker = Commands.runOnce(() -> {
        double dist = 0.0;
        if (currentPosition.equals("bottom")) {
            dist += ElevatorLiftConstants.bottomToAmpDistance;
        }

        elevator.runMotorsUntil("up", ElevatorLiftConstants.ampToSpeakerDistance + dist);
        currentPosition = "speaker";
    });

    public Command runMotorForwardWhile = Commands.runOnce(() -> {
        elevator.runMotorForwardWhile();
    });

    public Command runMotorReverseWhile = Commands.runOnce(() -> {
        elevator.runMotorReverseWhile();
    });

    public Command stopMotors = Commands.runOnce(() -> {
        elevator.stopMotors();
    });

    
}
