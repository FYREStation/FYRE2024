package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * The subsystem that will deal in everything vision related.
 */
public class VisionProcessing extends SubsystemBase {

    // Initialize the first camera object
    private final PhotonCamera camera1 = new PhotonCamera(VisionConstants.camera1); 

    // Initialize the second camera object
    private final PhotonCamera camera2 = new PhotonCamera(VisionConstants.camera2);


    // The output of the first camera
    private PhotonPipelineResult cam1Out;

    // The output of the second camera
    private PhotonPipelineResult cam2Out;

    public VisionProcessing() {}


    /**
     * Constatly grabs the result fromt the VP cameras.
     */
    @Override
    public void periodic() {
        cam1Out = camera1.getLatestResult();
        cam2Out = camera2.getLatestResult();
    }

    /**
     * Gets the first cameras result.

     * @return cam1Out - the pipeline result from the first camera
     */
    public PhotonPipelineResult getCam1Out() {
        return cam1Out;
    }

    /**
     * Gets the second cameras result.

     * @return cam2Out - the pipeline result from the second camera
     */
    public PhotonPipelineResult getCam2Out() {
        return cam2Out;
    }

    /**
     * Checks if a tag exists in the current pipeline result.

     * @return boolean - whether or not a tag is detected in the pipeline
     */
    public boolean tagExists() {
        return cam1Out.hasTargets();
    }

    /**
     * Returns the origin of the tag in 2d space relative to the camera feed.

     * @return coords - the x and y coordinates that make up the origin of the tag
     */
    public double[] getOrigin() {

        // returns null if a tag doesn't exist
        if (!tagExists()) {
            return null;
        }

        // defines the coordinate array for the origin
        double[] coords = new double[2];

        // gets the detected target and the array of corners detected
        PhotonTrackedTarget target = cam1Out.getBestTarget();
        List<TargetCorner> corners = target.getDetectedCorners();

        // averages the x coordinates
        coords[0] = (
            corners.get(0).x
            + corners.get(1).x
            + corners.get(2).x
            + corners.get(3).x
            ) / 4;

        // averages the y coordinates
        coords[1] = (
            corners.get(0).y
            + corners.get(1).y
            + corners.get(2).y
            + corners.get(3).y
            ) / 4;


        // returns the coordinates
        return coords;
    }
}
