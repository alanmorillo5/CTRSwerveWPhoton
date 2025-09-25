package frc.robot;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;

public final class Constants {
    
    public static final class CameraConstants {
        public static final double kFrontCameraToRobotX = -0.25019; 
        public static final double kFrontCameraToRobotY = 0.18685;
        public static final double kFrontCameraToRobotZ = -0.3604;

        public static final double kFrontCameraToRobotPitch = 0;
        public static final double kFrontCameraToRobotYaw = 0;
        public static final double kFrontCameraToRobotRoll = 0;

        public static final Transform3d kFrontCameraToRobotTransform = new Transform3d(
        new Translation3d(kFrontCameraToRobotX, kFrontCameraToRobotY, kFrontCameraToRobotZ),
        new Rotation3d(kFrontCameraToRobotPitch, kFrontCameraToRobotYaw, kFrontCameraToRobotRoll)
        );
    }
}
