// Vibhav: imports

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** The intake subsystem to be used by any intake commands. */
// Vibhav: creates inkate class and intake motors and related things
public class Intake extends SubsystemBase {

    // The redline motor that will spin the intake wheels.
    private final CANSparkMax intakeWheels = new CANSparkMax(
        IntakeConstants.intakeWheelPort, 
        CANSparkLowLevel.MotorType.kBrushed
    );

    // The neo motor that will handle the intake actuation.
    // Vibhav: creates actuation moter objects.
    private final CANSparkMax intakeActuation = new CANSparkMax(
        IntakeConstants.intakeActuationPort,
        CANSparkLowLevel.MotorType.kBrushed
    );

    // The encoder for the intake actuation.
    private final RelativeEncoder intakeEncoder = intakeActuation.getAlternateEncoder(
        IntakeConstants.intakeEncoderCount
    );

    /** Basic constructior to assign motor values and set encoders. */
    // Vibhav:intake encoder resest
    public Intake() {
        intakeEncoder.setPosition(0);
    }

    /**
     * Spings the intake wheels forward.
     *
     * @param speed - the direction for the motor to spin.
     */
    // Vibhav: spins the wheels (motors)
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
    // Vibhav: runs the motors until the encoder distance travels the distance
    public void runMotorsUntil(String direction, double distance) {
        double newPosition = getEncoder() + distance;
        double motorPower = direction == "down" ? -0.4 : 0.4;

        while (getEncoder() < newPosition) {
            intakeActuation.set(motorPower);
        }
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

    // Vibhav: returns the intake position
    public double getEncoder() {
        return intakeEncoder.getPosition();
    }

    /**
     *  Resets the encoder. The distance and position will be set to 0. 
     */

    // Vibhav: resets the encoder
    public void resetEncoder() {
        intakeEncoder.setPosition(0);
    }
}