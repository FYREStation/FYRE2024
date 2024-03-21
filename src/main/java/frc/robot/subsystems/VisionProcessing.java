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
    private final PhotonCamera camera = new PhotonCamera(VisionConstants.camera1); 


    // The output of the first camera
    private PhotonPipelineResult camOut;


    public VisionProcessing() {}


    /**
     * Constatly grabs the result fromt the VP cameras.
     */
    @Override
    public void periodic() {
        camOut = camera.getLatestResult();
    }

    /**
     * Gets the first cameras result.

     * @return cam1Out - the pipeline result from the first camera
     */
    public PhotonPipelineResult getCam1Out() {
        return camOut;
    }

    /**
     * Checks if a tag exists in the current pipeline result.

     * @return boolean - whether or not a tag is detected in the pipeline
     */
    public boolean tagExists() {
        return camOut.hasTargets();
    }

    public int getTagID() {
        return camOut.getBestTarget().getFiducialId();
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
        PhotonTrackedTarget target = camOut.getBestTarget();

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

    public double getArea() {
        return camOut.getBestTarget().getArea();
    }
}
