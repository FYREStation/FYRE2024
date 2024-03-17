package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.Constants.ManipulatorConstants;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.Climbing;
import frc.robot.commands.Driving;
import frc.robot.commands.ElevatorLift;
import frc.robot.commands.FaceApriltag;
import frc.robot.commands.IntakeControl;
import frc.robot.subsystems.Autonomous;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.VisionProcessing;

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

    // Initializes the elevator subsystem and command.
    private final Elevator elevator = new Elevator();
    private final ElevatorLift elevatorCommand = new ElevatorLift(elevator);

    // Initializes the intake subsystem and command.
    private final Intake intake = new Intake();
    private final IntakeControl intakeCommand = new IntakeControl(intake);

    // Initializes the climber subsystem and command.
    // private final Climber climber = new Climber();
    // private final Climbing climberCommand = new Climbing(climber);

    // Initializes the vision subsystem and command.
    private final VisionProcessing vision = new VisionProcessing();
    private final FaceApriltag visionCommand = new FaceApriltag(vision, driveTrain);

    // Initializes the autonomous subsystem and command.
    private final Autonomous autonomous = new Autonomous(new String[] {
        "paths/Forward.wpilib.json", 
        "paths/RedRight.wpilib.json"
    });
    private final AutoCommand autoCommand = new AutoCommand(autonomous, driveTrain, intakeCommand, elevator, visionCommand);

    // Initializes the autonomous chooser
    private final SendableChooser<Command> autoChooser = new SendableChooser<>();

    // Creates the xbox controller instance
    public static final CommandXboxController driverControl =
        new CommandXboxController(DriveTrainConstants.driverControlPort);

    // Creates the joystick instance
    public static final CommandJoystick manipulatorControl = 
        new CommandJoystick(ManipulatorConstants.manipulatorControlPort);


    // Values for the cameras plugged into the ROBORIO for driver vision
    // these are not used for apriltags
    private UsbCamera camera1;
    private UsbCamera camera2;

    /** The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
        // Sets default commands for all subsystems.
        driveTrain.setDefaultCommand(driveCommand);
        elevator.setDefaultCommand(elevatorCommand);
        intake.setDefaultCommand(intakeCommand);
        vision.setDefaultCommand(visionCommand);

        // sets and displays all of the auto options
        autoChooser.setDefaultOption("Forward", autoCommand.getAutonomousCommand(0));
        autoChooser.addOption("RedRight", autoCommand.getAutonomousCommand(1));

        SmartDashboard.putData(autoChooser);

        // Configure the trigger bindings
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
        manipulatorControl.button(8)
            .onTrue(elevatorCommand.goToTop);

        // toggles manaual elevator override
        manipulatorControl.button(9).onTrue(
            elevatorCommand.toggleManualOverride);

        // starts the calibration sequence
        manipulatorControl.button(6).onTrue(
            elevatorCommand.calibrateLiftBounds);

        // stops the elevator calibration
        manipulatorControl.button(4).onTrue(
            elevatorCommand.stopCalibration
        );

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

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    public Driving getDriveCommand() {
        return driveCommand;
    }
}
