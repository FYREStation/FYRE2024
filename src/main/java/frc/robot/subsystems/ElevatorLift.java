// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import frc.robot.Constants; 

/** The drivetrain subsystem to be used by any driving commands. */
public class ElevatorLift extends SubsystemBase {
    // The CIM which will be the "leader" for the elevator lift.
    private final CANSparkMax elevatorMotor1 = new CANSparkMax(
        Constants.ElevatorLiftConstants.elevatorMotor1Port, 
        CANSparkLowLevel.MotorType.kBrushless
    );

    // The other CIM on the elevator lift which will follow the main motor.
    private final CANSparkMax elevatorMotor2 = new CANSparkMax(
        Constants.ElevatorLiftConstants.elevatorMotor2Port,
        CANSparkLowLevel.MotorType.kBrushless
    );

    private final sun.awt.AWTCharset.Encoder elevatorEncoder = new Encoder(
        Constants.ElevatorLiftConstants.elevatorEncoderA, 
        Constants.ElevatorLiftConstants.elevatorEncoderB
    );

    /** Attaches the right motor to the left motor for ease of use. */ 
    public ElevatorLift() {
        elevatorMotor2.follow(elevatorMotor1);
        elevatorEncoder.reset();
    }

    /**
     * Returns the position of the elevator encoder. 
     *
     * @return - The integer value of the rotational position of the encoder.
     */
    public int getEncoder() {
        return elevatorEncoder.get();
    }
    
    /**
     * Returns the distance of the elevator encoder.
     *
     * @return - The integer value of the distance traveled by the encoder.
     */
    public int getEncoderDistance() {
        return elevatorEncoder.getDistance();
    }

    /**
     * Resets the encoder. The distance and position will be set to `0`=.
     */
    public void resetEncoder() {
        elevatorEncoder.reset();
    }
}
