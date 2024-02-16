// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriverConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Driving;
import frc.robot.commands.ElevatorLift;
import frc.robot.commands.IntakeControl;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
    // Initializes the drivetrain command and subsystem.
    private final DriveTrain driveTrain = new DriveTrain();
    private final Driving driveCommand = new Driving(driveTrain);

    // Initializes the elevator subsystem and command.
    private final Elevator elevator = new Elevator();
    private final ElevatorLift elevatorCommand = new ElevatorLift(elevator);

    // Initializes the intake subsystem and command.
    private final Intake intake = new Intake();
    private final IntakeControl intakeCommand = new IntakeControl(intake);

    // Initializes the autonomous subsystem and command.
    private final Autonomous autonomous = new Autonomous("paths/AutonomousForward.wpilib.json");
    private final AutoCommand autoCommand = new AutoCommand(autonomous, driveTrain);

    // Creates the xbox controller instance
    public static final CommandXboxController driverControl =
        new CommandXboxController(DriverConstants.driverControlPort);

    // Creates the joystick instance
    public static final CommandJoystick manipulatorControl =
        new CommandJoystick(ManipulatorConstants.manipulatorControlPort);

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {

        // sets default commands for all subsystems
        driveTrain.setDefaultCommand(driveCommand);
        elevator.setDefaultCommand(elevatorCommand);
        intake.setDefaultCommand(intakeCommand);
        autonomous.setDefaultCommand(autoCommand);

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

        // controls the toggle for the drivetrain
        driverControl.axisGreaterThan(3, 0.75)
            .onTrue(driveCommand.toggleSpeedOn)
            .onFalse(driveCommand.toggleSpeedOff);

        // controls the elevator
        manipulatorControl.button(8)
            .onTrue(elevatorCommand.goToBottom);
        manipulatorControl.button(10)
            .onTrue(elevatorCommand.stopMotors);
        manipulatorControl.button(12)
            .onTrue(elevatorCommand.goToAmp);

        manipulatorControl.button(7)
            .onTrue(elevatorCommand.runMotorForwardWhile)
            .onFalse(elevatorCommand.stopMotors);

        manipulatorControl.button(11)
            .onTrue(elevatorCommand.runMotorReverseWhile)
            .onFalse(elevatorCommand.stopMotors);

        // controls the intake spinning
        manipulatorControl.button(5)
            .onTrue(intakeCommand.outTakeNote)
            .onFalse(intakeCommand.stopIntake);;
        manipulatorControl.button(3)
            .onTrue(intakeCommand.intakeNote)
            .onFalse(intakeCommand.stopIntake);

        manipulatorControl.button(6)
            .onTrue(intakeCommand.intakeUp)
            .onFalse(intakeCommand.intakeStop);

        manipulatorControl.button(4)
            .onTrue(intakeCommand.intakeDown)
            .onFalse(intakeCommand.intakeStop);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @returns the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonomous.getDefaultCommand();
    }

    // public void resetElevatorEncoder() {
    //     elevator.resetEncoder();
    // }
}
