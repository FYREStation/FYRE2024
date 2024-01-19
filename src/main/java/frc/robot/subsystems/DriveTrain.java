// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

/** The drivetrain subsystem to be used by any driving commands. */
public class DriveTrain extends SubsystemBase {
    // Initializes the four motors used by the drivetrain.
    private final CANSparkMax left1 = 
        new CANSparkMax(Constants.DriveTrainConstants.left1MotorPort);

    private final CANSparkMax right1 = 
        new CANSparkMax(Constants.DriveTrainConstants.right1MotorPort);

    private final CANSparkMax left2 = 
        new CANSparkMax(Constants.DriveTrainConstants.left2MotorPort);

    private final CANSparkMax right2 = new 
        CANSparkMax(Constants.DriveTrainConstants.right2MotorPort);


    // Creates two motor controller groups for the two sides of the robot.
    private final MotorControllerGroup leftMotors = new MotorControllerGroup(left1, left2);
    private final MotorControllerGroup rightMotors = new MotorControllerGroup(right1, right2);

    // Initializes the differential drive for the robot.
    private final DifferentialDrive diffDrive = new DifferentialDrive(leftMotors, rightMotors);

    /** Initializes the DriveTrain subsystem by setting up motors. */
    public DriveTrain() {
        setupMotors(new CANSparkMax[]{left1, right1, left2, right2});
    }

    /**
     * Sets up the motors passed through as an array to have the proper safety
     * and timeout procedures.
     *
     * @param motors - An Array of `CANSparkMax` motor controllers.
     */
    public void setupMotors(CANSparkMax[] motors) {
        for (CANSparkMax motor : motors) {
            // TODO: figure out the equivilant values of these for the spark maxs and fix this
            motor.setSafetyEnabled(false);
            motor.setExpiration(99999);
        }

        diffDrive.setSafetyEnabled(true);
        diffDrive.setExpiration(99999);
    }

    /** 
     * Uses the tank drive mechanic to maneuver the robot's drivetrain. 

     * @param movementSpeed - The movement speed of the tank drive system.
     * @param rotationalSpeed - The rotational speed of the tank drive system.
     */
    public void tankDrive(double movementSpeed, double rotationalSpeed) {
        movementSpeed *= DriveTrainConstants.invertedDrive ? -1 : 1;
        diffDrive.tankDrive(movementSpeed, rotationalSpeed);
    }

    /** 
     * Uses the arcade drive mechanic to maneuver the robot's drivetrain. 

     * @param movementSpeedLeft - The left motor movement speed of the arcade drive system.
     * @param movementSpeedRight - The right motor movement speed of the arcade drive system.
     */
    public void arcadeDrive(double movementSpeedLeft, double movementSpeedRight) {
        int multiplier = DriveTrainConstants.invertedDrive ? -1 : 1;
        diffDrive.tankDrive(movementSpeedLeft, movementSpeedRight);
    }

    /**
     * Toggles whether tank or arcade drive will be used to drive the robot.
     * (there is a boolean value in Driving.java that you can use)
     * 
     * TODO: finish this method
     */
    public void toggleTankDrive() {

    }
}
