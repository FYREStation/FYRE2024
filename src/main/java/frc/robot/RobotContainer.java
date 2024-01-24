// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.Driving;
import frc.robot.commands.ElevatorLift;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveTrain driveTrain = new DriveTrain();
    private final Driving driveCommand = new Driving(driveTrain);
    
    private final Elevator elevator = new Elevator();
    private final ElevatorLift elevatorCommand = new ElevatorLift(elevator);

    // Creates the xbox controller instance
    public static final CommandXboxController driverControl =
        new CommandXboxController(DriverConstants.driverControlPort);

    // Creates the joystick instance
    public static final CommandJoystick manipulatorControl = 
        new CommandJoystick(ManipulatorConstants.manipulatorControlPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        driveTrain.setDefaultCommand(driveCommand);
        elevator.setDefaultCommand(elevatorCommand);
        
        // Configure the trigger bindings
        configureBindings();
    }

    /**
     * Use this method to define your trigger->command mappings. Triggers can be created via the
     * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
     * predicate, or via the named factories in {@link
     * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
     * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
     * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
     * joysticks}.
     */
    private void configureBindings() {
        // Toggles the tank drive mode when the a button is pressed on the xbox controller
        driverControl.a().onTrue(driveCommand.toggleDriveTrain);

        manipulatorControl.button(8).onTrue(elevatorCommand.goToBottom);
        manipulatorControl.button(10).onTrue(elevatorCommand.goToAmp);
        manipulatorControl.button(12).onTrue(elevatorCommand.goToSpeaker);
    }
    
    /*
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    // public Command getAutonomousCommand() {
    //     // An example command will be run in autonomous
    //     return Autos.exampleAuto(exampleSubsystem);
    // }
}
