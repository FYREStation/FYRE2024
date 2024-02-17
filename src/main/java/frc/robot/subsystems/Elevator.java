// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Vibhav: imports

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ElevatorLiftConstants;

/** The elevator subsystem to be used by any elevator commands. */
// Vibhav: creates the elevator class and the elevator moters and related things
public class Elevator extends ProfiledPIDSubsystem {
    // The CIM which will be the "leader" for the elevator lift.
    private final CANSparkMax elevatorMotor1 = new CANSparkMax(
        ElevatorLiftConstants.elevatorMotor1Port, 
        CANSparkLowLevel.MotorType.kBrushless
    );

    // The other CIM on the elevator lift which will follow the main motor.
    // Vibhav: creates support moter objects.
    private final CANSparkMax elevatorMotor2 = new CANSparkMax(
        ElevatorLiftConstants.elevatorMotor2Port,
        CANSparkLowLevel.MotorType.kBrushless
    );

    // The encoder on one of the elevator cims.
    private final RelativeEncoder elevatorEncoder1 = elevatorMotor1.getEncoder();

    // The encoder on one of the elevator cims.
    private final RelativeEncoder elevatorEncoder2 = elevatorMotor2.getEncoder();

    private final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(
        ElevatorLiftConstants.staticGain,
        ElevatorLiftConstants.gravityGain,
        ElevatorLiftConstants.velocityGain
    );

    /** Attaches the right motor to the left motor for ease of use. */ 
    // Vibhav: attaches the motors to each other
    public Elevator() {
        // invokes the constructor of the inherited class
        super(
            // creates a new PID controller for the elevator
            new ProfiledPIDController(
                ElevatorLiftConstants.kP,
                ElevatorLiftConstants.kI,
                ElevatorLiftConstants.kD,
                new TrapezoidProfile.Constraints(
                    ElevatorLiftConstants.maxVelocity,
                    ElevatorLiftConstants.maxAcceleration
                )
            )
        );

        // initalizes the motors
        setUpMotors();
    }

    /**
     * Runs the elevator motors until the encoder distance travels {@code distance} units.
     *
     * @param direction - The direction for the motor to travel; the {@code "down"} tag will
     *     make the motors run down, and vice versa.
     * 
     * @param distance - The distance for the motors to travel.
     */
    public void runMotorsUntil(String direction, double distance) {
        double newPosition = getEncoderDistances() + distance;
        double motorPower =
            direction == "down"
            ? -ElevatorLiftConstants.elvevatorThrottle
            : ElevatorLiftConstants.elvevatorThrottle;

        while (getEncoderDistances() < newPosition) {
            elevatorMotor1.set(motorPower);
        }
    }

    /**
     * Sets up the motors at the beginning of the program.
     */
    public void setUpMotors() {
        // sets the second elevator motor to be the inverse of the first
        elevatorMotor2.follow(elevatorMotor1, true);

        // reset the encoder values
        resetEncoders();
    }


    /**
     * Returns the position of the elevator encoder. 
     *
     * @return - The integer value of the rotational position of the encoder.
     */
    // Vibhav: returns the elevator position
    public int getEncoder() {
        // what is this supposed to do?
        return 0;
    }

    // /**
    //  * Returns the distance of the elevator encoder.
    //  *
    //  * @return - The double value of the distance traveled by the encoder.
    //  */
    // Vibhav: returns the distance of the elevator
    public double getEncoderDistances() {
        return (elevatorEncoder1.getPosition() + -elevatorEncoder2.getPosition()) / 2;
    }

    // /**
    //  * Resets the encoder. The distance and position will be set to `0`=.
    //  */
    // Vibhav: resets the encoder
    public void resetEncoders() {
        elevatorEncoder1.setPosition(0);
        elevatorEncoder2.setPosition(0);
    }


    /** 
     * Runs the elevator motors up
     */
    public void runMotorForwardWhile() {
        elevatorMotor1.set(ElevatorLiftConstants.elvevatorThrottle);
    }

    /**
     * Runs the elevator motors down
     */
    public void runMotorReverseWhile() {
        elevatorMotor1.set(-ElevatorLiftConstants.elvevatorThrottle);
    }

    /**
     * Stops the motors
     */
    public void stopMotors()  {
        elevatorMotor1.set(0.0);
    }

    /**
     * Overriden method from the super class
     * This will take in the output, and a set point,
     * and calculates the amout the motor needs to spin based on this input
     */
    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = elevatorFeedForward.calculate(setpoint.position, setpoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        elevatorMotor1.setVoltage(output + feedforward);
    }


    @Override
    protected double getMeasurement() {
        // possibly add an offset here? 
        return getEncoderDistances();
    }
}
