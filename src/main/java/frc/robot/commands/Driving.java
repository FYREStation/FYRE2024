// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DriveTrain;

/** The driving functionality for our robot using the drivetrain. */
public class Driving extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain driveTrain;

    // Initialize the move speed variables.
    private double leftMovementSpeed;
    private double rightMovementSpeed;

    // Initialize our speed variables for controlling motor speeds.
    private double leftStick;
    private double rightStick;

    // Initialize the tank drive toggle value. 
    private boolean isTank;

    // Fetch the driver controller from the RobotContainer.
    private CommandXboxController driverControl;
    
    /**
     * Creates a new Driving command based on a DriveTrain subsystem.
     *
     * @param subsystem - The DriveTrain subsystem to be integrated into the command.
     */
    public Driving(DriveTrain subsystem) {
        this.driveTrain = subsystem;
        addRequirements(subsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Set the driverControl variable to our XboxController.
        driverControl = RobotContainer.driverControl;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // Get the values of the joysticks we will use for our particular drive.
        leftStick = isTank ? -driverControl.getLeftY() : driverControl.getLeftY();
        rightStick = isTank ? driverControl.getRightY() : driverControl.getRightX(); 

        // Calculates the power to apply to each set of motors. 
        leftMovementSpeed = leftStick * DriveTrainConstants.throttle;
        rightMovementSpeed = rightStick * DriveTrainConstants.throttle;

        // Runs each set of motors based on their calculated power levels. 
        if (isTank) {
            driveTrain.tankDrive(leftMovementSpeed, rightMovementSpeed);
        } else {
            driveTrain.arcadeDrive(rightMovementSpeed, leftMovementSpeed);
        }
    }

    // Toggles the isTank value, switching the robot from tank to 
    // arcade drive and vice versa.
    public Command toggleDriveTrain = Commands.runOnce(() -> {
        isTank = !isTank;
    });
}
