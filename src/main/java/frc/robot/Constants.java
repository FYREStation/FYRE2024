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
    public static final double trackWidthMeters = 0.5207;

    /** Initializes the autonomous constants. */
    public static class AutonomousConstants {

        // PID CONTROLLER VALUES //

        // the proportion value for the PID controller
        public static final double kP = 0.00275;

        // the integral value for the PID controller
        public static final double kI = 0.0005;
        // 2

        // the derivative value for the PID controller
        public static final double kD = 0.00075;


        // MOVEMENT VALUES //

        // The maximum acceleration of the robot in meters.
        public static final double kMaximumAcceleration = 0.025;

        // The maximum speed of the robot in meters.
        public static final double kMaximumVelocity = 0.0125;

        // FEEDFORWARD VALUES //
        // The values below can be found in the robot diagnostics.`

        // The volts that the robot is pulling (kS in feedforward).
        public static final double kSfeedforward = 0.075;

        // The volts per second that the robot is pulling (kV in feedforward).
        public static final double kVfeedforward = .3;

        // The volts per second squared that the robot is pulling (kA in feedforward).
        public static final double kAfeedforward = 0.06;
    }

    /** Initializes the drive train constants. */
    public static class DriveTrainConstants {

        // CONTROLLER VALUES //

        // The port on the laptop that the driver controler will exist on
        public static final int driverControlPort = 0;

        // The throttle for the drivetrain; all input motor values will be multipled by this value.
        public static final double throttle = 1;

        // The limit constant; currently unused.
        public static final double limitConstant = 0.4;

        // The controller acceleration limit
        public static final double accelerationLimit = 0.05;

        // The deadband for the drivetrain. Values sent under this boundary will be set to 0.
        public static final double deadband = 0.25;

        // DRIVER MOTOR VALUES //

        // The motor port for the first left motor.
        public static final int left1MotorPort = 1;

        // The motor port for the first right motor.
        public static final int right1MotorPort = 2;

        // The motor port for the second left motor.
        public static final int left2MotorPort = 3;

        // The motor port for the second right motor.
        public static final int right2MotorPort = 4;

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
        public static final double elvevatorThrottle = 0.3;

        // LIMIT SWITCH VALUES //

        public static final int bottomLimitSwitchPort = 9;

        public static final int topLimitSwitchPort = 8;


        // FEEDFORWARD VALUES //

        // The static gain of the elevator controller
        public static final double staticGain = 0.4;

        // The gravity gain of the elevator controller
        public static final double gravityGain = 0.25;

        // The velocity gain of the elevator controller
        public static final double velocityGain = 0.47;

        public static final double maxVelocity = 160;

        public static final double maxAcceleration = 60;

        // PID VALUES //

        // the proportion value for the PID controller
        public static final double kP = 0.065;

        // the integral value for the PID controller
        public static final double kI = 0.000275;
        // 2

        // the derivative value for the PID controller
        public static final double kD = 0.00775;
    }

    /** Initializes the intake mechanism constants. */
    public static class IntakeConstants {
        // INTAKE MOTOR VALUES //

        // The motor port of the intake wheels.
        public static final int intakeWheelPort = 8;


        // The motor port of the intake actuation.
        public static final int intakeActuationPort = 9;

        // The intake wheel spin throttle
        public static final double intakeWheelThrottle = 0.35;

        // The intake actuation throttle
        public static final double intakeActuationThrottle = 0.5;

        // ENCODER VALUES //

        // The a channel of the intake encoder
        public static final int intakeEncoderA = 0;

        // The b channel of the intake encoder
        public static final int intakeEncoderB = 1;

        // FEEDFORWARD VALUES //

        // The static gain of the elevator controller
        public static final double staticGain = 2.5;

        // The gravity gain of the elevator controller
        public static final double gravityGain = 0.0;

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
        // 0.1
    }

    /** Initialized the climber constants.*/
    public static class ClimberConstants {
        // The port of the climber motor.
        public static final int climberMotorPort = 10;

        // The throttle for the climber motor.
        public static final double climberThrottle = 0.25;
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
