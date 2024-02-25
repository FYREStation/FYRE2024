// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.ElevatorLiftConstants;

/** The elevator subsystem to be used by any elevator commands. */
public class Elevator extends ProfiledPIDSubsystem {

    // The CIM which will be the "leader" for the elevator lift.
    private final CANSparkMax elevatorMotor1 = new CANSparkMax(
        ElevatorLiftConstants.elevatorMotor1Port, 
        CANSparkLowLevel.MotorType.kBrushless
    );

    // The other CIM on the elevator lift which will follow the main motor.
    private final CANSparkMax elevatorMotor2 = new CANSparkMax(
        ElevatorLiftConstants.elevatorMotor2Port,
        CANSparkLowLevel.MotorType.kBrushless
    );

    // The feedforward loop 
    private final ElevatorFeedforward elevatorFeedForward = new ElevatorFeedforward(
        ElevatorLiftConstants.staticGain,
        ElevatorLiftConstants.gravityGain,
        ElevatorLiftConstants.velocityGain
    );

    // The encoder on one of the elevator cims.
    private final RelativeEncoder elevatorEncoder1 = elevatorMotor1.getEncoder();

    // The encoder on one of the elevator cims.
    private final RelativeEncoder elevatorEncoder2 = elevatorMotor2.getEncoder();

    // The limit switch at the bottom of the elevator
    private final DigitalInput bottomLimitSwitch = new DigitalInput(
        ElevatorLiftConstants.bottomLimitSwitchPort
    );

    // The limit switch at the top of the elevator
    private final DigitalInput topLimitSwitch = new DigitalInput(
        ElevatorLiftConstants.topLimitSwitchPort
    );

    // The profile for the top position of the elevator
    private TrapezoidProfile.State topState = new TrapezoidProfile.State(100, 0);

    // The profile for the bottom position of the elevator
    private TrapezoidProfile.State bottomState = new TrapezoidProfile.State(0, 0);

    // The variable that will be used to calculate the maximum rotations to the top of the elevator from the bottom
    private double rotationsToTop = 38;

    // Variable to keep track of if the elevator can move up.
    private boolean canMoveUp = true;

    // Variable to keep track of if the elevator can move down.
    private boolean canMoveDown = false;

    /** Attaches the right motor to the left motor for ease of use. */ 
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

    @Override
    public void periodic() {
        // gets the applied current to the elevator motor
        double appliedCurrent = elevatorMotor1.getOutputCurrent();
        if (appliedCurrent > 0 && (getTopSwitch() || getEncoderDistances() >= rotationsToTop)) {
            canMoveUp = false;
        } else {
            canMoveUp = true;
        }

        if (appliedCurrent < 0 && (getBottomSwitch() || getEncoderDistances() <= 0)) {
            canMoveDown = false;
        } else {
            canMoveDown = true;
        }
    }

    /**
     * Sets up the motors at the beginning of the program.
     */
    private void setUpMotors() {
        // sets the second elevator motor to be the inverse of the first
        elevatorMotor2.follow(elevatorMotor1, true);

        // reset the encoder values
        resetEncoders();
    }

    /** 
     * Runs the elevator motors up.
     */
    public void runMotorForward() {
        if (canMoveUp) {
            elevatorMotor1.set(ElevatorLiftConstants.elvevatorThrottle);
        } else {
            elevatorMotor1.stopMotor();
        }
    }

    /**
     * Runs the elevator motors down.
     */
    public void runMotorReverse() {
        if (canMoveDown) {
            elevatorMotor1.set(-ElevatorLiftConstants.elvevatorThrottle);
        } else {
            elevatorMotor1.stopMotor();
        }

    }

    /**
     * Stops the motors in manual and PID control systems.
     */
    public void stopMotors()  {
        elevatorMotor1.set(0.0);
        disable();
    }

    /**
     * Returns the distance of the elevator encoder.
     *
     * @return - The double value of the distance traveled by the encoder.
     */
    public double getEncoderDistances() {
        return (elevatorEncoder1.getPosition() + elevatorEncoder2.getPosition()) / 2;
    }

    /**
     * Resets the encoder. The distance and position will be set to '0'.
     */
    public void resetEncoders() {
        elevatorEncoder1.setPosition(0);
        elevatorEncoder2.setPosition(0);
    }

    /**
     * Returns the state of the botom limit switch on the elevator
     * This will return true if it is being triggered, and false if not.

     * @return switch value - the value of the limit switch
     */
    public boolean getBottomSwitch() {
        return bottomLimitSwitch.get();
    }

    /**
     * Returns the state of the top limit switch on the elevator
     * This will return true if it is being triggered, and false if not.

     * @return switch value - the value of the limit switch
     */
    public boolean getTopSwitch() {
        return topLimitSwitch.get();
    }

    /**
     * Returns the top state of the elevator.

     * @return topState - the top state of the elevator
     */
    public TrapezoidProfile.State getUpState() {
        return topState;
    }

    /**
     * Returns the bottoms state of the elevator.

     * @return bottomState - the bottom state of the elevator
     */
    public TrapezoidProfile.State getDownState() {
        return bottomState;
    }

    /**
     * Calibrates the elevator by running the motor from the bottom position to the top,
     * and measuring the encoder values from that point.
     */
    public void calibrateElevatorBounds() {

        // ensures that the encoders are at the bottom of the elevator
        while (!getBottomSwitch()) {
            elevatorMotor1.set(-0.1);
        }
        elevatorMotor1.stopMotor();

        // resets the encoder values at the bottom
        resetEncoders();

        // runs the motors to the top of the elevator
        while (!getTopSwitch()) {
            elevatorMotor1.set(0.1);
        }
        elevatorMotor1.stopMotor();

        // saves the rotational value at the top
        rotationsToTop = getEncoderDistances();

        // lowers the elevator back down
        while (!getBottomSwitch()) {
            elevatorMotor1.set(-0.1);
        }
        elevatorMotor1.stopMotor();
    }

    /**
     * This will take in the output, and a set point,
     * and calculates the amout the motor needs to spin based on this input.
     */
    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = elevatorFeedForward.calculate(setpoint.position, setpoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        elevatorMotor1.setVoltage(output + feedforward);
    }


    /**
     * Method to be used by the PID controller under the hood,
     * this is not used in our code but it is essential to PID.
     * DO NOT DELETE THIS METHOD
     */
    @Override
    protected double getMeasurement() {
        // possibly add an offset here? 
        return getEncoderDistances();
    }
}
