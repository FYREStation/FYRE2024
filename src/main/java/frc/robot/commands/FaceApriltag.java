package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionProcessing;

/** This turns the robot to face the best apriltag it can find. */
public class FaceApriltag extends Command {

    private VisionProcessing vision;
    private DriveTrain drive;

    double[] tagOrigin;

    /** Initialize a new apriltag command. */
    public FaceApriltag(VisionProcessing subsystem, DriveTrain drive) {
        this.vision = subsystem;
        this.drive = drive;
        addRequirements(subsystem, drive);
    }


    public Command findTag = Commands.run(() -> {
        tagOrigin = vision.getOrigin();
        if (tagOrigin != null) {
            drive.tankDrive((tagOrigin[0] - (VisionConstants.camResolution[0] / 2)) / 600, (tagOrigin[0] - (VisionConstants.camResolution[0] / 2)) / 600);
            System.out.println(tagOrigin[0]);
        } else {
            drive.tankDrive(0, 0);
            System.out.println("Valid Tag Not Found");
        }
    });

}
