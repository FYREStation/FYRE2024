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

    // The variable that will be used to calculate the maximum rotations to the top of the elevator
    private double rotationsToTop = 38;

    // The profile for the top position of the elevator
    private TrapezoidProfile.State topState = new TrapezoidProfile.State(rotationsToTop, 0);

    // The profile for the bottom position of the elevator
    private TrapezoidProfile.State bottomState = new TrapezoidProfile.State(0, 0);

    // Variable to control if the driver needs to manually override the elevator
    private boolean manualOverride = false;

    // Variable to keep track of if the elevator can move up.
    private boolean canMoveUp = true;

    // Variable to keep track of if the elevator can move down.
    private boolean canMoveDown = false;

    // Variable to track if the elevator is calibrating
    private boolean isCalibrating = false;

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
        // checks if the controller is not yet at it's goal and the manual override is not active
        if (!super.getController().atGoal() && !manualOverride && !isCalibrating) {
            // sets the motor to the calculated value by the controller
            elevatorMotor1.set(
                super.getController().calculate(
                    getEncoderDistances(),
                    super.getController().getGoal()
                )
            );
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
     * Sets the goal to the top state of the elevator.
     */
    public void goToTop() {
        setGoal(topState);
        enable();
    }

    /**
     * Sets the goal to the bottom state of the elevator.
     */
    public void goToBottom() {
        setGoal(bottomState);
        enable();
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
    public boolean calibrateStep1() {
        // ensures that the encoders are at the bottom of the elevator
        if (!getBottomSwitch()) {
            elevatorMotor1.set(-0.1);
            return false;
        }
        elevatorMotor1.stopMotor();

        // resets the encoder values at the bottom
        resetEncoders();
        return true;
    }

    /**
     * The second step of the elevator calibration.
     * Causes the elevator to go all the way up to the top

     * @return boolean - whether or not the step has been completed
     */
    public boolean calibrateStep2() {
        // runs the motors to the top of the elevator
        if (!getTopSwitch()) {
            elevatorMotor1.set(0.1);
            return false;
        }
        elevatorMotor1.stopMotor();

        // saves the rotational value at the top
        rotationsToTop = getEncoderDistances();
        return true;
    }

    /**
     * The third step of the elevator calibration.
     * Causes the elevator to go back down.

     * @return boolean - whether this step has completed or not
     */
    public boolean calibrateStep3() {
        // lowers the elevator back down
        if (!getBottomSwitch()) {
            elevatorMotor1.set(-0.1);
            return false;
        }
        elevatorMotor1.stopMotor();
        return true;
    }

    /**
     * Sets the elevator calibration state.
     */
    public void setCalibrating(boolean isCalibrating) {
        this.isCalibrating = isCalibrating;
    }

    /**
     * Toggles the manaul ovrride control of the elevator.
     */
    public void toggleManualOverride() {
        manualOverride = !manualOverride;
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
