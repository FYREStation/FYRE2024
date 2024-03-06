// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    // ROBOT DIMENSIONS //
    public static final double trackWidthMeters = 0.5576;

    /** Initializes the autonomous constants. */
    public static class AutonomousConstants {

        // PID CONTROLLER VALUES //

        // The proportional coeffecient
        public static final double kP = 0.1;

        // The integral coefficient
        public static final double kI = 0.1;

        // The derivative coefficient
        public static final double kD = 0.1;

        // MOVEMENT VALUES //

        // The maximum acceleration of the robot in meters.
        public static final double kMaximumAcceleration = 1.0;

        // The maximum speed of the robot in meters.
        public static final double kMaximumVelocity = 3.0;

        // FEEDFORWARD VALUES //
        // The values below can be found in the robot diagnostics.

        // The volts that the robot is pulling (kS in feedforward).
        public static final double kSfeedforward = 0.22;

        // The volts per second that the robot is pulling (kV in feedforward).
        public static final double kVfeedforward = 1.98;

        // The volts per second squared that the robot is pulling (kA in feedforward).
        public static final double kAfeedforward = 0.2;
    }

    /** Initializes the drive train constants. */
    public static class DriveTrainConstants {

        // CONTROLLER VALUES //

        // The throttle for the drivetrain; all input motor values will be multipled by this value.
        public static final double throttle = 1.0;

        // The limit constant; currently unused.
        public static final double limitConstant = 0.4;

        // The controller acceleration limit
        public static final double accelerationLimit = 0.05;

        // The deadband for the drivetrain. Values sent under this boundary will be set to 0.
        public static final double deadband = 0.25;

        // DRIVER MOTOR VALUES //

        // The motor port for the first left motor.
        public static final int left1MotorPort = 4;

        // The motor port for the first right motor.
        public static final int right1MotorPort = 1;

        // The motor port for the second left motor.
        public static final int left2MotorPort = 2;

        // The motor port for the second right motor.
        public static final int right2MotorPort = 3;

        // A boolean value to specify inverted drive.
        public static final boolean invertedDrive = false;
    }

    /** Initializes the elevator lift constants. */
    public static class ElevatorLiftConstants {
        // ELEVATOR MOTOR VALUES //

        // The motor port for the first elevator motor.
        public static final int elevatorMotor1Port = 5;

        // The motor port for the second elevator motor.
        public static final int elevatorMotor2Port = 6;

        // The elevator motor throttle value
        public static final double elvevatorThrottle = 0.35;

        // ENCODER VALUES //

        public static final int bottomLimitSwitchPort = 9;

        public static final int topLimitSwitchPort = 8;

        // The A channel for the elevator encoder.
        public static final int elevatorEncoderA = 0;

        // The B channel for the elevator encoder.
        public static final int elevatorEncoderB = 1;

        // The distance-per-pulse value of the encoder.
        public static final int encoderPulseDistance = 8;

        // The distance from the bottom position of the lift to the amp position.
        public static final double bottomToAmpDistance = 0.275634;

        // The distance from the amp position of the lift to the speaker position.
        public static final double ampToSpeakerDistance = 0.47253;

        // LIMIT SWITCH VALUES //




        // FEEDFORWARD VALUES //

        // The static gain of the elevator controller
        public static final double staticGain = 2.5;

        // The gravity gain of the elevator controller
        public static final double gravityGain = 0.30;

        // The velocity gain of the elevator controller
        public static final double velocityGain = 0.05;

        public static final double maxVelocity = 0.25;

        public static final double maxAcceleration = 0.025;

        // PID VALUES //

        // the proportion value for the PID controller
        public static final double kP = 0.7;

        // the integral value for the PID controller
        public static final double kI = 0;
        // 2

        // the derivative value for the PID controller
        public static final double kD = 0;
    }

    /** Initializes the intake mechanism constants. */
    public static class IntakeConstants {
        // INTAKE MOTOR VALUES //

        // The motor port of the intake wheels.
        public static final int intakeWheelPort = 9;

        // The motor port of the intake actuation.
        public static final int intakeActuationPort = 8;

        // The intake wheel spin throttle
        public static final double intakeThrottle = 1.0;

        // ENCODER VALUES //

        // The counts per revolution of the encoder
        public static final int intakeEncoderCount = 4;

        // The distance from the bottom position of the lift to the amp position.
        public static final double bottomToAmpDistance = 0.275634;

        // The distance from the amp position of the lift to the speaker position.
        public static final double ampToSpeakerDistance = 0.47253;
    }

    /** Initializes the vision constants. */
    public static class VisionConstants {

        /** The string name for the first camera. */
        public static final String camera1 = "Cam1";

        /** The string name for the second camera. */
        public static final String camera2 = "Cam2";

        // The resolution of the cameras
        public static double[] camResolution = {800, 600};
    }

    /** Initializes the driver constants. */
    public static class DriverConstants {
        // The port of the Xbox Controller.
        public static final int driverControlPort = 0; 
    }

    /** Initializes the manipulator constants. */
    public static class ManipulatorConstants {
        // Initialize the joystick to port one.
        public static final int manipulatorControlPort = 1;
    }
}
