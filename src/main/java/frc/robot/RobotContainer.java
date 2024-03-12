// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Vibhav: this is the imports from other files

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Driving;
import frc.robot.commands.ElevatorLift;
import frc.robot.commands.FaceApriltag;
import frc.robot.commands.IntakeControl;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionProcessing;
import frc.robot.subsystems.Climber;
import frc.robot.commands.Climbing;

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

    // Initializes the elevator subsystem and command.
    private final Elevator elevator = new Elevator();
    private final ElevatorLift elevatorCommand = new ElevatorLift(elevator);

    // Initializes the intake subsystem and command.
    private final Intake intake = new Intake();
    private final IntakeControl intakeCommand = new IntakeControl(intake);

    // Initializes the climber subsystem and command.
    private final Climber climber = new Climber();
    private final Climbing climberCommand = new Climbing(climber);

    // Initializes the vision subsystem and command.
    private final VisionProcessing vision = new VisionProcessing();
    private final FaceApriltag visionCommand = new FaceApriltag(vision, driveTrain);

    // Initializes the autonomous subsystem and command.
    private final Autonomous autonomous = new Autonomous("paths/AutonomousLine.wpilib.json");

    // Creates the xbox controller instance
    // Vibhav: not much to say here... ^^^
    public static final CommandXboxController driverControl =
        new CommandXboxController(DriveTrainConstants.driverControlPort);

    // Creates the joystick instance
    // Vibhav: not much to say here... ^^^
    public static final CommandJoystick manipulatorControl = 
        new CommandJoystick(ManipulatorConstants.manipulatorControlPort);


    // Values for the cameras plugged into the ROBORIO for driver vision
    // these are not used for apriltags
    private UsbCamera camera1;
    private UsbCamera camera2;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    // Vibhav: sets default code
    public RobotContainer() {
        // Sets default commands for all subsystems.
        driveTrain.setDefaultCommand(driveCommand);
        elevator.setDefaultCommand(elevatorCommand);
        intake.setDefaultCommand(intakeCommand);
        vision.setDefaultCommand(visionCommand);
     
        // Configure the trigger bindings
        // Vibhav: configures connection buttons --> commands
        configureBindings();

        camera1 = CameraServer.startAutomaticCapture(0);
        camera2 = CameraServer.startAutomaticCapture(1);
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
        // toggles the tank drive mode when the a button is pressed on the xbox controller
        driverControl.a()
            .onTrue(driveCommand.toggleDriveTrain);

            /* 
        driverControl.povUp()
            .onTrue(climberCommand.climbUp)
            .onFalse(climberCommand.stopClimb);

        driverControl.povDown()
            .onTrue(climberCommand.reverseClimb)
            .onFalse(climberCommand.stopClimb);
            */


        // controls the toggle for the drivetrain
        driverControl.axisGreaterThan(2, 0.75)
            .whileTrue(visionCommand.findTag);

        // controls the toggle for the drivetrain.
        driverControl.axisGreaterThan(3, 0.75)
            .onTrue(driveCommand.toggleSpeedOn)
            .onFalse(driveCommand.toggleSpeedOff);

        // PID elevator control
        manipulatorControl.button(12)
            .onTrue(elevatorCommand.goToBottom);
        manipulatorControl.button(10)
            .onTrue(elevatorCommand.stopMotors);
        manipulatorControl.button(8)
            .onTrue(elevatorCommand.goToTop);

        // calibrates the elevator
        manipulatorControl.button(9).onTrue(
            elevatorCommand.calibrateLiftBounds);

        // manual elevator control
        manipulatorControl.button(7)
            .onTrue(elevatorCommand.runMotorForward)
            .onFalse(elevatorCommand.stopMotors);
        manipulatorControl.button(11)
            .onTrue(elevatorCommand.runMotorReverse)
            .onFalse(elevatorCommand.stopMotors);

        // controls the intake spinning
        manipulatorControl.button(1)
            .onTrue(intakeCommand.intakeNote)
            .onFalse(intakeCommand.stopIntakeWheels);
        manipulatorControl.button(2)
            .onTrue(intakeCommand.outTakeNote)
            .onFalse(intakeCommand.stopIntakeWheels);

        // controls the intake actuation
        manipulatorControl.button(5)
            .onTrue(intakeCommand.intakeUp)
            .onFalse(intakeCommand.stopIntakeActuation);
        manipulatorControl.button(3)
            .onTrue(intakeCommand.intakeDown)
            .onFalse(intakeCommand.stopIntakeActuation);
    }
    
    /*
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        Trajectory traj = autonomous.getAutonomousTrajectory();

        System.out.println("traj made");
        System.out.println(traj);
        
        return new AutoCommand(autonomous, driveTrain).getAutonomousCommand();
    }
}
