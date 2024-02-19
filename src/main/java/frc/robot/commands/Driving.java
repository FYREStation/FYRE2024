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

import java.lang.Math;

/** The driving functionality for our robot using the drivetrain. */
public class Driving extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain driveTrain;

    // Initialize the move speed variables.
    private double leftMovementSpeed;
    private double rightMovementSpeed;

    // Initialize our speed variables for controlling motor speeds.
    private double leftStick = 0;
    private double rightStick = 0;

    private double prevLeft;
    private double prevRight;

    // Initialize the tank drive toggle value. 
    private boolean isTank = true;

    // The speed of the drivetrain when the throttle isn't being pressed
    private double driveSpeedLimit = 0.75;

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
        prevLeft = leftStick;
        prevRight = rightStick;
        // Get the values of the joysticks we will use for our particular drive.
        leftStick = isTank ? driverControl.getRightY() : driverControl.getLeftY();
        rightStick = isTank ? - driverControl.getLeftY() : driverControl.getRightX(); 
        
        //limitAcceleration();

        // Calculates the power to apply to each set of motors. 
        leftMovementSpeed = leftStick * DriveTrainConstants.throttle * driveSpeedLimit;
        rightMovementSpeed = rightStick * DriveTrainConstants.throttle * driveSpeedLimit;

        // Runs each set of motors based on their calculated power levels. 
        if (isTank) {
            driveTrain.tankDrive(leftMovementSpeed, rightMovementSpeed);
        } else {
            driveTrain.arcadeDrive(rightMovementSpeed, leftMovementSpeed);
        }
    }


    /**
     * This method will limit the ammount that the sticks register in any direction.
     * This does cause some issues when trying to turn or decellerate, so don't use until you are out of options.
     */
    private void limitAcceleration() {
        if (Math.abs(leftStick) - Math.abs(prevLeft) > DriveTrainConstants.accelerationLimit) {
            if (leftStick >= 0) {
                leftStick = prevLeft + DriveTrainConstants.accelerationLimit;
            } else {
                leftStick = prevLeft - DriveTrainConstants.accelerationLimit;
            }
        }
        if (Math.abs(rightStick) - Math.abs(prevRight) > DriveTrainConstants.accelerationLimit) {
            if (rightStick >= 0) {
                rightStick = prevRight + DriveTrainConstants.accelerationLimit;
            } else {
                rightStick = prevRight - DriveTrainConstants.accelerationLimit;
            }
        }
    }

    // Toggles the isTank value, switching the robot from tank to 
    // arcade drive and vice versa.
    public Command toggleDriveTrain = Commands.runOnce(() -> {
        isTank = !isTank;
    });

    // Toggles the speed limit to go full throttle.
    public Command toggleSpeedOn = Commands.runOnce(() -> {
        driveSpeedLimit = 1;
    });

    // Toggles the speed limit to go 75% speed.
    public Command toggleSpeedOff = Commands.runOnce(() -> {
        driveSpeedLimit = 0.75;
    });
}
