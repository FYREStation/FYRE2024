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
        CANSparkLowLevel.MotorType.kBrushless
    );

    // The neo motor that will handle the intake actuation.
    private final CANSparkMax intakeActuation = new CANSparkMax(
        IntakeConstants.intakeActuationPort,
         CANSparkLowLevel.MotorType.kBrushless
    );

    // The encoder for the intake actuation.
    private final RelativeEncoder intakeEncoder = intakeActuation.getAlternateEncoder(
        IntakeConstants.intakeEncoderCount
    );

    /** Basic constructior to assign motor values and set encoders. */
    public Intake() {
        intakeEncoder.setPosition(0);
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
     * Runs the elevator motors until the encoder distance travels {@code distance} units.
     *
     * @param direction - The direction for the motor to travel; the {@code "down"} tag will
     *     make the motors run down, and vice versa.
     * 
     * @param distance - The distance for the motors to travel.
     */
    public void runMotorsUntil(String direction, double distance) {
        double newPosition = getEncoder() + distance;
        double motorPower = direction == "down" ? -0.4 : 0.4;

        while (getEncoder() < newPosition) {
            intakeActuation.set(motorPower);
        }
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

}