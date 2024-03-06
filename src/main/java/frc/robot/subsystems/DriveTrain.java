// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.SPI;
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

    // Instantiates the gyroscope.
    public AHRS ahrsGyro;

    private final DifferentialDriveOdometry diffOdometry;
    private final RelativeEncoder leftEncoder = leftMotor1.getEncoder();
    private final RelativeEncoder rightEncoder = rightMotor1.getEncoder();

    /** Initializes the DriveTrain subsystem by setting up motors. */
    public DriveTrain() {
        // Sets up the main motors and the differential drive.
        setupMotors();

        // initializes and calibrates the gyro
        ahrsGyro = new AHRS(SPI.Port.kMXP);
        resetAhrs();

        // sets up differential drive odometry
        diffOdometry = new DifferentialDriveOdometry(
            ahrsGyro.getRotation2d(), 
            leftEncoder.getPosition(), 
            rightEncoder.getPosition()
        );
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

        diffDrive.setMaxOutput(0.90);

        // Sets up safety measures for the other motors.
        diffDrive.setSafetyEnabled(true);
        diffDrive.setExpiration(0.1);
        diffDrive.feed();
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
        diffDrive.arcadeDrive(movementSpeed, -rotationalSpeed);
    }

    /**
     * Fetches the angle of the gyroscope.
     *
     * @return - The angle of the gyroscope.
     */
    public double getGyroscope() {
        return ahrsGyro.getAngle();
    }

    /**
     * Resets the positioning of the gyroscope.
     */
    public void resetAhrs() {
        ahrsGyro.reset();
    }

    /** Updates the odometry calculations every scheduler run. */
    @Override
    public void periodic() {
        diffOdometry.update(
            ahrsGyro.getRotation2d(),
            leftEncoder.getPosition(), 
            rightEncoder.getPosition()
        );
        diffDrive.feed();
    }

    /**
     * Returns the current pose of the robot, converted
     * from the robot's odomoetry.
     *
     * @return - The pose of the robot.
     */
    public Pose2d getPose() {
        return diffOdometry.getPoseMeters();
    }

    /**
     * Returns the current velocity of each set of wheels.
     *
     * @return - A DifferentialDriveWheelSpeeds wrapper containing
     *     the speed of each set of wheels.
     */
    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(
            leftEncoder.getVelocity(), 
            rightEncoder.getVelocity()
        );
    }

    /**
     * Resets the odomoetry of the robot and sets it to the 
     * passed in pose.
     *
     * @param pose - The pose to reset the robot to.
     */
    public void resetOdometry(Pose2d pose) {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);

        diffOdometry.resetPosition(
            ahrsGyro.getRotation2d(), 
            leftEncoder.getPosition(), 
            rightEncoder.getPosition(), 
            pose
        );
    }

    /**
     * Sets the voltage of each set of motors to a specified value.
     *
     * @param leftVolts - The voltage for the left set of wheels.
     * @param rightVolts - The voltage for the right set of wheels.
     */
    public void tankDriveVolts(double leftVolts, double rightVolts) {
        leftMotor1.setVoltage(leftVolts);
        rightMotor1.setVoltage(rightVolts);
        diffDrive.feed();
    }
}
