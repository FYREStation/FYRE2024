package frc.robot.subsystems;

import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

/** The intake subsystem to be used by any intake commands. */
public class Intake extends SubsystemBase{
    // The redline motor that will spin the intake wheels.
    private final CANSparkMax intakeWheels = new CANSparkMax(
        IntakeConstants.intakeWheelPort, 
        CANSparkLowLevel.MotorType.kBrushless
    );
}
