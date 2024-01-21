// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveTrainConstants;

/** The drivetrain subsystem to be used by any driving commands. */
public class DriveTrain extends SubsystemBase {
    // Initializes the four motors used by the drivetrain.


    // the other motor for the left drive will 'follow' this motor
    private final CANSparkMax leftMotors = 
        new CANSparkMax(Constants.DriveTrainConstants.left1MotorPort, 
        CANSparkLowLevel.MotorType.kBrushless);

    // the other motor for the right drive will 'follow' this motor
    private final CANSparkMax rightMotors = 
        new CANSparkMax(Constants.DriveTrainConstants.right1MotorPort,
        CANSparkLowLevel.MotorType.kBrushless);

    private final CANSparkMax left2 = 
        new CANSparkMax(Constants.DriveTrainConstants.left2MotorPort,
        CANSparkLowLevel.MotorType.kBrushless);

    private final CANSparkMax right2 = new 
        CANSparkMax(Constants.DriveTrainConstants.right2MotorPort,
        CANSparkLowLevel.MotorType.kBrushless);

    // Initializes the differential drive for the robot.
    private DifferentialDrive diffDrive;

    /** Initializes the DriveTrain subsystem by setting up motors. */
    public DriveTrain() {
        // set up main motors
        setupMotors();

        // set a deadband for the d
        diffDrive.setDeadband(DriveTrainConstants.DEADBAND);
    }

    /**
     * Sets up the motors passed through as an array to have the proper safety
     * and timeout procedures.
     *
     */
    public void setupMotors() {
        // creates the psuedo motor group
        left2.follow(leftMotors);
        right2.follow(rightMotors);

        // sets up differential drive
        diffDrive = new DifferentialDrive(leftMotors, rightMotors);

        // sets safety measures for the motors
        diffDrive.setSafetyEnabled(false);
        diffDrive.setExpiration(0.1);
        diffDrive.feed();
    }

    /** 
     * Uses the tank drive mechanic to maneuver the robot's drivetrain. 

     * @param movementSpeedLeft - The movement speed of the left side of the drive.
     * @param movementSpeedRight - The movement speed of the right side of the drive.
     */
    public void tankDrive(double movementSpeedLeft, double movementSpeedRight) {
        diffDrive.tankDrive(movementSpeedLeft, movementSpeedRight);
    }

    /** 
     * Uses the arcade drive mechanic to maneuver the robot's drivetrain. 

     * @param movementSpeed - The movement speed of the drive system.
     * @param rotationalSpeed - The rotational speed of the drive system.
     */
    public void arcadeDrive(double movementSpeed, double rotationalSpeed) {
        diffDrive.arcadeDrive(movementSpeed, rotationalSpeed);
    }
}
