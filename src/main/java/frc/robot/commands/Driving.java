// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveTrainConstants;
import frc.robot.subsystems.DriveTrain;

/** An example command that uses an example subsystem. */
public class Driving extends Command {
    @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
    private final DriveTrain driveTrain;

    // Initialize the move speed variables.
    private double leftMovementSpeed;
    private double rightMovementSpeed;

    // Initialize our speed variables for controlling motor speeds.
    private double leftStick;
    private double rightStick;

    // Initialize the boolean to control the tank drive toggle
    private boolean isTank;


    // Fetch the driver controller from the RobotContainer.
    private CommandXboxController driverControl;
    

    /**
     * Creates a new ExampleCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public Driving(DriveTrain subsystem) {
        this.driveTrain = subsystem;
        // Use addRequirements() here to declare subsystem dependencies.
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


        System.out.println(leftStick + " : left, " + rightStick + " : right"); 


        // Apply a deadband to the speed modifiers if they are negligible. 
        double[] speeds = new double[]{leftStick, rightStick};
        speeds = deadband(speeds);

        // Perform calculations to limit the turning power. 
        double sum = Math.abs(leftStick + rightStick); // 0 - 2;
        double average = (sum / 2) * DriveTrainConstants.LIMIT_CONSTANT; // 0 - 0.3; 
        double limitedValue = 1 - average; // 0.7 - 1; 

        // Calculates the power to apply to each set of motors. 
        leftMovementSpeed = leftStick * DriveTrainConstants.THROTTLE;
        rightMovementSpeed = rightStick * DriveTrainConstants.THROTTLE;

        // Outputs the positions of each of the joystick axis. 
        // System.out.println(leftStick + " : left stick, " + rightStick + " : right stick"); 
        // System.out.println(leftPower + " : left power, " + rightPower + " : right power"); 

        // Runs each set of motors based on their calculated power levels. 
        if (isTank) {
            driveTrain.tankDrive(leftMovementSpeed, rightMovementSpeed);
        } else {
            driveTrain.arcadeDrive(rightMovementSpeed, leftMovementSpeed);
        }
    }

    /**
     * Checks if the displacement of each axis is sufficient enough for movement.

     * @param speeds - An array of speeds to compare to the deadband constant.
     */
    public double[] deadband(double[] speeds) { 
        for (int i = 0; i < speeds.length; i++) {
            if (Math.abs(speeds[i]) < DriveTrainConstants.CONTROLLER_DEADBAND) {
                speeds[i] = 0.0;
            }
        }

        return speeds;
    }
}
