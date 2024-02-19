package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;


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


    @Override
    public void periodic() {
        cam1Out = camera1.getLatestResult();
        cam2Out = camera2.getLatestResult();
    }

    public PhotonPipelineResult getCam1Out() {
        return cam1Out;
    }

    public PhotonPipelineResult getCam2Out() {
        return cam2Out;
    }

}
