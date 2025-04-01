package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Dictionary;
import java.util.Hashtable;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import swervelib.SwerveDrive;

/* relevant sources:
 * https://docs.wpilib.org/en/2022/docs/software/kinematics-and-odometry/swerve-drive-odometry.html
 * https://docs.photonvision.org/en/v2025.2.1/docs/examples/poseest.html
 * https://docs.wpilib.org/en/stable/docs/software/advanced-controls/state-space/state-space-pose-estimators.html#
 * https://www.chiefdelphi.com/t/apriltag-odometry-with-photonvision-help/451360
 * https://github.com/lasarobotics/PH2024/blob/master/src/main/java/frc/robot/subsystems/vision/VisionSubsystem.java
 * https://docs.photonvision.org/en/latest/docs/programming/photonlib/robot-pose-estimator.html
 */

public class VisionSubsystem {

    // camera location on robot (meters)
    Transform3d robotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0,0,0)); //Cam mounted facing forward, half a meter forward of center, half a meter up from center.

    // april tags layout
    AprilTagFieldLayout aprilTagFieldLayout;
    // ArduCam
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

    public VisionSubsystem(){
        try {
            aprilTagFieldLayout = new AprilTagFieldLayout("./2025-reefscape-andymark.json");
        } catch (IOException e) {
            System.err.println("VISION FILE PATH IS WRONG");
        }
    }

    public void setOdometryWithVision(SwerveDrive mSwerveDrive) {
        // TODO: choose pose strategy
        PhotonPoseEstimator photonPoseEstimator = new PhotonPoseEstimator(aprilTagFieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCam);
        var result = camera.getLatestResult();
        boolean photon1HasTargets = result.hasTargets();
        if (photon1HasTargets) {
            var poseEstimate = photonPoseEstimator.update(result);
            Pose3d currentPose3d = poseEstimate.get().estimatedPose;
            Pose2d botPose = currentPose3d.toPose2d();
            double photonTimestamp = poseEstimate.get().timestampSeconds;

            mSwerveDrive.addVisionMeasurement(botPose, photonTimestamp); // TODO get std devs
        }
    }



}