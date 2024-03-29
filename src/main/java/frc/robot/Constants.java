// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    /** Initializes the drive train constants. */
    public static class DriveTrainConstants {
        // Initialize the xbox controller 
        public static final int driverControlPort = 0; 
        public static final double THROTTLE = 1.0;
        public static final double LIMIT_CONSTANT = 0.4;
        public static final double DEADBAND = 0.25;

        // The motor port for the first left motor.
        public static final int left1MotorPort = 0;

        // The motor port for the first right motor.
        public static final int right1MotorPort = 1;

        // The motor port for the second left motor.
        public static final int left2MotorPort = 2;

        // The motor port for the second right motor.
        public static final int right2MotorPort = 3;

        // A boolean value to specify inverted drive.
        public static final boolean invertedDrive = false;
    }

    /** Initializes the manipulator constants. */
    public static class ManipulatorConstants {
        // Initialize the joystick to port one.
        public static final int manipulatorControlPort = 0;
    }
}
