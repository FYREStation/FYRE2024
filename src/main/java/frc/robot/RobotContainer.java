// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Vibhav: this is the imports from other files

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.Driving;
import frc.robot.subsystems.DriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */

// Vibhav: this creates objects of the classes to use later in file
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final DriveTrain driveTrain = new DriveTrain();
    private final Driving driveCommand = new Driving(driveTrain);

    // Creates the xbox controller instance
    // Vibhav: not much to say here... ^^^
    public static final CommandXboxController driverControl =
        new CommandXboxController(DriveTrainConstants.driverControlPort);

    // Creates the joystick instance
    // Vibhav: not much to say here... ^^^
    public static final CommandJoystick manipulatorControl = 
        new CommandJoystick(ManipulatorConstants.manipulatorControlPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    // Vibhav: sets default code
    public RobotContainer() {
        driveTrain.setDefaultCommand(driveCommand);
        
        // Configure the trigger bindings
        // Vibhav: configures connection buttons --> commands
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

    // Vibhav: toggles tank controls.
    private void configureBindings() {
        // Toggles the tank drive mode when the a button is pressed on the xbox controller
        driverControl.a().onTrue(driveCommand.toggleDriveTrain);
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
