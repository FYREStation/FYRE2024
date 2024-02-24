package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
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

        intakeEncoder.reset();
        intakeActuation.setInverted(true);
        intakeWheels.setInverted(false);
    }

    /**
     * Spings the intake wheels forward.
     *
     * @param speed - the direction for the motor to spin.
     */
    public void spinWheels(double speed) {
        intakeWheels.set(speed);
    }

    /**
     * Runs the actuation at a set speed.

     * @param speed - the speed to set the intake.
     */
    public void runActuation(double speed) {
        intakeActuation.set(speed);
    }

    /**
     * Returns the position of the intake encoder.
     *
     * @return - The integer value of the rotational position of the encoder.
     */
    public double getEncoder() {
        return intakeEncoder.getDistance();
    }

    /**
     *  Resets the encoder. The distance and position will be set to 0. 
     */
    public void resetEncoder() {
        intakeEncoder.reset();
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
        return getEncoder();
    }
}