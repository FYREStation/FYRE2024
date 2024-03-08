package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Climber;

public class Climbing extends Command {
    private Climber climber;

    public Climbing(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    public Command climbUp = Commands.run(() -> {
        climber.climb();
    });

    public Command stopClimb = Commands.run(() -> {
        climber.stopClimb();
    });

    public Command reverseClimb = Commands.run(() -> {
        climber.reverseClimb();
    });

    
}