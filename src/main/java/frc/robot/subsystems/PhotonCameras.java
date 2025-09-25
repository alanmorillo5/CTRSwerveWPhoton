package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class PhotonCameras extends SubsystemBase{
    private PhotonCamera frontCam;

    private PhotonPipelineResult frontCamResult;

    private AprilTagFieldLayout layout;

    private Transform3d frontCamToRobot;

    Timer timer;

    double[] x_poses;
    double[] y_poses;
    double[] theta_poses;
    int counter;

    public PhotonCameras(AprilTagFieldLayout layout) {
        frontCam = new PhotonCamera("frontCam");

        frontCamToRobot = new Transform3d();

        timer = new Timer();
        timer.start();

        frontCamResult = null;
        this.layout = layout;
        counter = 0;
        x_poses = new double[300];
        y_poses = new double[300];
        theta_poses = new double[300];
    }

    public void periodic() {

        frontCamResult = frontCam.getLatestResult();
    }

    public Matrix<N3, N1> getStandardDevs() {
        Matrix<N3, N1> stdDevs = new Matrix<N3, N1>(Nat.N3(), Nat.N1());
        stdDevs.set(1, 1, StandardDev(x_poses));
        stdDevs.set(2, 1, StandardDev(y_poses));
        stdDevs.set(3, 1, StandardDev(theta_poses));
        return stdDevs;
    }

    private double StandardDev(double[] test) {
        double mean_sum = 0;
        for(int i = 0; i < test.length; i++) {
            mean_sum += test[i];
        }
        double mean = mean_sum/(double)(test.length);

        double var_sum = 0;
        for(int i = 0; i < test.length; i++) {
            var_sum += Math.pow(test[i] - mean, 2);
        }
        return Math.sqrt(var_sum/(double)(test.length - 1));
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
            Transform3d test_pose = target.getBestCameraToTarget();
            x_poses[counter] = test_pose.getX();
            y_poses[counter] = test_pose.getY();
            theta_poses[counter] = test_pose.getRotation().getZ();
            counter = (counter + 1) % 300;
            return robotPose.toPose2d();
        }
        return null;
    }

    public int getBestFrontCamFiducialId() {
        if(!frontCamHasTarget() || frontCamResult == null) {
            return -1;
        }
        return frontCamResult.getBestTarget().getFiducialId();
    }
}
