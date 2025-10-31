package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CameraConstants;


public class PhotonCameras extends SubsystemBase{
    private PhotonCamera frontCam;

    private PhotonPipelineResult frontCamResult;

    private AprilTagFieldLayout layout;

    private Transform3d frontCamToRobot;

    private Timer timer;
    private double latestTimeStamp;

    public PhotonCameras(AprilTagFieldLayout layout) {
        frontCam = new PhotonCamera(CameraConstants.kFrontCameraName);

        frontCamToRobot = CameraConstants.kFrontCameraToRobotTransform;

        timer = new Timer();
        timer.start();
        latestTimeStamp = timer.get();

        frontCamResult = null;
        this.layout = layout;
    }

    public void periodic() {

        frontCamResult = frontCam.getLatestResult();
    }

    public boolean frontCamHasTarget() {
        if(frontCamResult == null) {
            return false;
        }
        return frontCamResult.hasTargets();
    }

    public double getFrontArea() {
        if (!frontCamHasTarget() || frontCamResult == null) {
            return 0;
        }
        return frontCamResult.getBestTarget().getArea();
    }

    public Pose2d getPoseRelativeFrontCam() {
        if(!frontCamHasTarget() || frontCamResult == null) {
            return null;
        }
        PhotonTrackedTarget target = frontCamResult.getBestTarget();

        if(layout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                layout.getTagPose(target.getFiducialId()).get(),
                frontCamToRobot
            );
            latestTimeStamp = timer.get();
            return robotPose.toPose2d();
        }
        return null;
    }

    public double getLatestTimeStamp() {
        return latestTimeStamp;
    }

    public int getBestFrontCamFiducialId() {
        if(!frontCamHasTarget() || frontCamResult == null) {
            return -1;
        }
        return frontCamResult.getBestTarget().getFiducialId();
    }
}
