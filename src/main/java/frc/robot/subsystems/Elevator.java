// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

// Vibhav: imports

package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ElevatorLiftConstants; 


/** The elevator subsystem to be used by any elevator commands. */
// Vibhav: creates the elevator class and the elevator moters and related things
public class Elevator extends SubsystemBase {
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
    // private final Encoder elevatorEncoder = new Encoder(
    //     ElevatorLiftConstants.elevatorEncoderA, 
    //     ElevatorLiftConstants.elevatorEncoderB
    // );

    /** Attaches the right motor to the left motor for ease of use. */ 
    // Vibhav: attaches the motors to each other
    public Elevator() {
        elevatorMotor2.follow(elevatorMotor1, true);
        // elevatorEncoder.reset();

        // elevatorEncoder.setDistancePerPulse(ElevatorLiftConstants.encoderPulseDistance);
    }

    /**
     * Runs the elevator motors until the encoder distance travels {@code distance} units.
     *
     * @param direction - The direction for the motor to travel; the {@code "down"} tag will
     *     make the motors run down, and vice versa.
     * 
     * @param distance - The distance for the motors to travel.
    //  */
    // public void runMotorsUntil(String direction, double distance) {
    //     double newPosition = getEncoderDistance() + distance;
    //     double motorPower = direction == "down" ? -0.1 : 0.1;

    //     while (getEncoderDistance() < newPosition) {
    //         elevatorMotor1.set(motorPower);
    //     }
    // }

    /**
     * Returns the position of the elevator encoder. 
     *
     * @return - The integer value of the rotational position of the encoder.
     */
    // Vibhav: returns the elevator position
    // public int getEncoder() {
    //     return elevatorEncoder.get();
    // }

    // /**
    //  * Returns the distance of the elevator encoder.
    //  *
    //  * @return - The double value of the distance traveled by the encoder.
    //  */

    // Vibhav: returns the distance of the elevator
    // public double getEncoderDistance() {
    //     return elevatorEncoder.getDistance();
    // }

    // /**
    //  * Resets the encoder. The distance and position will be set to `0`=.
    //  */

    // Vibhav: resets the encoder
    // public void resetEncoder() {
    //     elevatorEncoder.reset();
    // }

    public void runMotorForwardWhile() {
        elevatorMotor1.set(-0.2);
    }

    public void runMotorReverseWhile() {
        elevatorMotor1.set(0.2);
    }

    public void stopMotors()  {
        elevatorMotor1.set(0.0);
    }
}
