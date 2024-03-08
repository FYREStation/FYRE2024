package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

class Climbing extends CommandBase {
    private final Climber climber;

    public Climbing(Climber climber) {
        this.climber = climber;
        addRequirements(climber);
    }

    public Command climbUp = Commands.run(() -> {
        climber.climb()
    });

    public Command stopClimb = Commands.run(() -> {
        climber.stopClimb()
    });

    public Command reverseClimb = Commands.run(() -> {
        climber.reverseClimb()
    });

    
}