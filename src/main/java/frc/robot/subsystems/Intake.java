package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** The intake subsystem to be used by any intake commands. */
public class Intake extends SubsystemBase {

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

    // The encoder for the intake actuation.
    private final RelativeEncoder intakeEncoder = intakeActuation.getAlternateEncoder(
        IntakeConstants.intakeEncoderCount
    );

    /** Basic constructior to assign motor values and set encoders. */
    public Intake() {
        intakeEncoder.setPosition(0);
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
     * 
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
        return intakeEncoder.getPosition();
    }

    /**
     *  Resets the encoder. The distance and position will be set to 0. 
     */
    public void resetEncoder() {
        intakeEncoder.setPosition(0);
    }

    public void runMotorsUntil(String direction, double speed) {
        return;
    }
}