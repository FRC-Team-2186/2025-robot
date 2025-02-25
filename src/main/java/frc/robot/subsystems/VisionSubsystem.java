package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.math.geometry.Transform3d;

public class VisionSubsystem {

    // limelight
    private PhotonCamera camera = new PhotonCamera("photonvision"); // TODO: pass in actual camera name

    // current april tag id of target we're interested in
    private int curTargetId = 0;

    // current targets visible
    List<PhotonTrackedTarget> curTargets;

    // state vars containing info regarding target (skew not supported)
    private double targetYaw = 0.0;
    private double targetPitch = 0.0;
    private double targetArea = 0.0;
    Transform3d targetPose;
    List<TargetCorner> targetCorners;

    // have we seen any new input since the last call?
    public boolean cameraIsProcessingImages() {
        var result = camera.getAllUnreadResults();
        return !result.isEmpty();
    }

    // are we able to see any april tags currently?
    public boolean aprilTagsDetected() {
        var result = camera.getLatestResult();
        curTargets = result.getTargets();
        return result.hasTargets();
    }

    // do we see the april tag we're interested in? if so set relevant values in dict
    // TODO: determine what info we want and if we want to use simulations
    // Note: PID is a must for this to be useful
    public boolean desiredTargetInView(int tagId, List<PhotonTrackedTarget> targets) {
        for (PhotonTrackedTarget t : targets) {
            if (t.fiducialId == tagId) {
                targetPitch = t.getPitch();
                targetYaw = t.getYaw();
                targetArea = t.getArea();
                targetCorners = t.getDetectedCorners();
                targetPose = t.getBestCameraToTarget();
                return true;
            }
        }
        return false;
    }

}
