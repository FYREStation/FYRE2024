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

    // The left motor which will be the "leader" for all left motors.
    private final CANSparkMax leftMotor1 = new CANSparkMax(
        Constants.DriveTrainConstants.left1MotorPort, 
        CANSparkLowLevel.MotorType.kBrushless
    );

    // The right motor which will be the "leader" for all right motors.
    private final CANSparkMax rightMotor1 = new CANSparkMax(
        Constants.DriveTrainConstants.right1MotorPort,
        CANSparkLowLevel.MotorType.kBrushless
    );

    // The other left motor for the drivetrain. Controlled by leftMotor1.
    private final CANSparkMax leftMotor2 = new CANSparkMax(
        Constants.DriveTrainConstants.left2MotorPort,
        CANSparkLowLevel.MotorType.kBrushless
    );

    // The other right motor for the drivetrain. Controlled by rightMotor1.
    private final CANSparkMax rightMotor2 = new CANSparkMax(
        Constants.DriveTrainConstants.right2MotorPort,
        CANSparkLowLevel.MotorType.kBrushless
    );

    // Initializes the differential drive for the robot.
    private DifferentialDrive diffDrive;

    /** Initializes the DriveTrain subsystem by setting up motors. */
    public DriveTrain() {
        // Sets up the main motors and the differential drive.
        setupMotors();
    }

    /**
     * Sets up the motors passed through as an array to have the proper safety
     * and timeout procedures.
     */
    public void setupMotors() {
        // Attaches the "other" drivetrain motors to the "leader" motors.
        leftMotor2.follow(leftMotor1);
        rightMotor2.follow(rightMotor1);

        // Initializes the differential drive with the leader motors.
        diffDrive = new DifferentialDrive(leftMotor1, rightMotor1);

        // Sets up safety measures for the other motors.
        diffDrive.setSafetyEnabled(true);
        diffDrive.setExpiration(99999);
        diffDrive.setDeadband(DriveTrainConstants.deadband);
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
