package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
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

    // The feed forward controller for the elevator
    private final ArmFeedforward armFeedforward = new ArmFeedforward(
        IntakeConstants.staticGain,
        IntakeConstants.gravityGain,
        IntakeConstants.velocityGain
    );

    private final DigitalInput intakeSwitch = new DigitalInput(0);

    // The encoder for the intake actuation.
    private Encoder intakeEncoder = new Encoder(
        2,
        IntakeConstants.intakeEncoderB,
        false, 
        Encoder.EncodingType.k4X
    );

    // The number of rotations the motor must do to reach the bottom of the intake
    private double rotationsToBottom = 100;

    // The profile for the top position of the elevator
    private TrapezoidProfile.State topState = new TrapezoidProfile.State(0, 0);

    // The profile for the bottom position of the elevator
    private TrapezoidProfile.State bottomState = new TrapezoidProfile.State(-rotationsToBottom, 0);

    // Variable to keep track of if the intake can move down
    private boolean canMoveDown = true;

    // Variable to keep track of if the intake can move up.
    private boolean canMoveUp = true;

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
        //System.out.println(getSwitch());
    }

    /**
     * Sets up the motors at the beginning of the program.
     */
    private void setUpMotors() {
        intakeEncoder.reset();
        intakeEncoder.setDistancePerPulse(1);
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
     * Stops the intake wheels from spinning.
     */
    public void stopIntakeWheels() {
        intakeWheels.stopMotor();
    }

    /**
     * Runs the actuation at a set speed.
     */
    public void runActuationUp() {
        if (canMoveUp) {
            intakeActuation.set(IntakeConstants.intakeActuationThrottle);
        } else {
            intakeActuation.stopMotor();
        }

    }

    /**
     * Runs the actuation at a set speed.
     */
    public void runActuationDown() {
        if (canMoveDown) {
            intakeActuation.set(-IntakeConstants.intakeActuationThrottle);
        } else {
            intakeActuation.stopMotor();
        }

    }

    /**
     * Stops the motors in manual and PID control.
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

    public boolean getSwitch() {
        return intakeSwitch.get();
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