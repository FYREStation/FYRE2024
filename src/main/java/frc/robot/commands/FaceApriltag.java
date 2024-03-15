package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.VisionConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.VisionProcessing;

/** This turns the robot to face the best apriltag it can find. */
public class FaceApriltag extends Command {

    // initializes the vision subsystem
    private VisionProcessing vision;

    // initializes the drive subsystem
    private DriveTrain drive;

    // defines the cooridinate array for the tag
    double[] tagOrigin;

    /** Initialize a new apriltag command. */
    public FaceApriltag(VisionProcessing subsystem, DriveTrain drive) {
        this.vision = subsystem;
        this.drive = drive;
        addRequirements(subsystem);
    }

    /**
     * This command will turn the robot to face the apriltag closest to robot.
     * IT will only move if a valid tag is found.
     */
    public Command findTag = Commands.run(() -> {
        // gets the tag origin from the VP subsystem
        tagOrigin = vision.getOrigin();

        // checks to make sure that a tag exists
        if (tagOrigin != null) {

            // sets the robot to turn twards the tag
            // at a speed based on it's distance from the center of the screen
            drive.arcadeDrive(
                0,
                (tagOrigin[0] - (VisionConstants.camResolution[0] / 2))
                    / (VisionConstants.camResolution[0] / 2));

            // prints the origin
            System.out.println(tagOrigin[0]);
        } else {
            // tells the robot to stop and tells that a tag isn't found
            drive.tankDrive(0, 0);
            System.out.println("Valid Tag Not Found");
        }
    });

}
