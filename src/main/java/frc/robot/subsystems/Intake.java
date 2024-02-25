package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants.IntakeConstants;

/** The intake subsystem to be used by any intake commands. */
public class Intake extends ProfiledPIDSubsystem {

    // The redline motor that will spin the intake wheels.
    private final CANSparkMax intakeWheels = new CANSparkMax(
        IntakeConstants.intakeWheelPort, 
        CANSparkLowLevel.MotorType.kBrushed
    );

    // The neo motor that will handle the intake actuation.
    private final CANSparkMax intakeActuation = new CANSparkMax(
        IntakeConstants.intakeActuationPort,
        CANSparkLowLevel.MotorType.kBrushed
    );

    private final ArmFeedforward armFeedforward = new ArmFeedforward(
        IntakeConstants.staticGain,
        IntakeConstants.gravityGain,
        IntakeConstants.velocityGain
    );

    // The encoder for the intake actuation.
    private final Encoder intakeEncoder = new Encoder(
        IntakeConstants.intakeEncoderA,
        IntakeConstants.intakeEncoderB);

    // The profile for the top position of the elevator
    private TrapezoidProfile.State topState = new TrapezoidProfile.State(0, 0);

    // The profile for the bottom position of the elevator
    private TrapezoidProfile.State bottomState = new TrapezoidProfile.State(-100, 0);

    private double rotationsToBottom = 100;

    /** Basic constructior to assign motor values and set encoders. */
    public Intake() {
        super(
            // creates a new PID controller for the elevator
            new ProfiledPIDController(
                IntakeConstants.kP,
                IntakeConstants.kI,
                IntakeConstants.kD,
                new TrapezoidProfile.Constraints(
                    IntakeConstants.maxVelocity,
                    IntakeConstants.maxAcceleration
                )
            )
        );

        // initializes the motors
        setUpMotors();
    }

    @Override
    public void periodic() {
        // gets the applied current to the intake actuation
        double appliedCurrent = intakeActuation.getOutputCurrent();
        // if (
        //     // checks if the motor is trying to run the intake out of bounds
        //     (appliedCurrent > 0 && getEncoderDistance() <= -10)
        //     || (appliedCurrent < 0 && getEncoderDistance() >= rotationsToBottom)
        //     // if the intake tries to overstep, stop it
        //     ) stopAcutation();
    }

    /**
     * Sets up the motors at the beginning of the program
     */
    private void setUpMotors() {
        intakeEncoder.reset();
        intakeActuation.setInverted(true);
        intakeWheels.setInverted(false);
    }

    /**
     * Spings the intake wheels forward.
     */
    public void intakeNote() {
        intakeWheels.set(IntakeConstants.intakeWheelThrottle);
    }

    /**
     * Spings the intake wheels in reverse.
     */
    public void outTakeNote() {
        intakeWheels.set(-IntakeConstants.intakeWheelThrottle);
    }

    /**
     * Stops the intake wheels from spinning
     */
    public void stopIntakeWheels() {
        intakeWheels.stopMotor();
    }

    /**
     * Runs the actuation at a set speed.
     */
    public void runActuationUp() {
        intakeActuation.set(IntakeConstants.intakeActuationThrottle);
    }

    /**
     * Runs the actuation at a set speed.
     */
    public void runActuationDown() {
        intakeActuation.set(-IntakeConstants.intakeActuationThrottle);
    }

    /**
     * Stops the motors in manual and PID control
     */
    public void stopAcutation() {
        intakeActuation.stopMotor();
        disable();
    }

    /**
     * Returns the position of the intake encoder.

     * @return - The integer value of the rotational position of the encoder.
     */
    public double getEncoderDistance() {
        return intakeEncoder.getDistance();
    }

    /**
     *  Resets the encoder. The distance and position will be set to 0. 
     */
    public void resetEncoder() {
        intakeEncoder.reset();
    }

    /**
     * Returns the top state of the intake.

     * @return topState - the top state of the intake
     */
    public TrapezoidProfile.State getUpState() {
        return topState;
    }

    /**
     * Returns the bottoms state of the intake.

     * @return bottomState - the bottom state of the intake
     */
    public TrapezoidProfile.State getDownState() {
        return bottomState;
    }

    /**
     * This will take in the output, and a set point,
     * and calculates the amout the motor needs to spin based on this input.
     */
    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        // Calculate the feedforward from the sepoint
        double feedforward = armFeedforward.calculate(setpoint.position, setpoint.velocity);

        // Add the feedforward to the PID output to get the motor output
        intakeActuation.setVoltage(output + feedforward);
    }

    /**
     * Method to be used by the PID controller under the hood,
     * this is not used in our code but it is essential to PID.
     * DO NOT DELETE THIS METHOD
     */
    @Override
    protected double getMeasurement() {
        return getEncoderDistance();
    }
}