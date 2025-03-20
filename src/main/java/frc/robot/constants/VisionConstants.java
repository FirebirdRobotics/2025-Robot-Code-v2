package frc.robot.constants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;

public class VisionConstants {
    public static final String kaprilCamRightName = "AprilCamRight"; // Arducam_OV2311USB_Camera (1) Red
    public static final String kaprilCamLeftName = "AprilCamLeft"; // Arducam_OV2311USB_Camera (1) Blue

    public static final String kobjectCamName = "ObjectCam"; // Arducam_OV2311USB_Camera

    // Cam mounted 6 in left, 10.647 in forward, and 8 in up @ 0.9731071 deg pitch and 116.3762958 yaw
    public static final Transform3d kRobotToRightAprilCam =
            new Transform3d(new Translation3d(-0.2704338,0.1524, 0.2032), new Rotation3d(0, 0.0169839228695, 2.0311495329798-Units.degreesToRadians(10)));//+Units.degreesToRadians(180)));
    // Cam mounted 4 in left, 10.647 in forward, and 8 in up @ 0.9731071 deg pitch and 61.3762958 yaw
    public static final Transform3d kRobotToLeftAprilCam =
            new Transform3d(new Translation3d(-0.2704338,0.1016, 0.2032), new Rotation3d(0, 0.0169839228695, 1.071218444385-.3));//-Units.degreesToRadians(180)));
    // Cam mounted TBD
    public static final Transform3d kRobotToObjectCam =
            new Transform3d(new Translation3d(-4,10.647, 0.5), new Rotation3d(0, 0, 0));

/*Stuff for pantherbotics auto align *************************************************************** */
        public static final Transform2d kLeftTransform = new Transform2d(Units.inchesToMeters(20), Units.inchesToMeters(-6.5), Rotation2d.fromDegrees(-180));
        public static final Transform2d kRightTransform = new Transform2d(Units.inchesToMeters(20), Units.inchesToMeters(6.5), Rotation2d.fromDegrees(-180));
        public static final Transform2d kCenterTransform = new Transform2d(Units.inchesToMeters(17), Units.inchesToMeters(0), Rotation2d.fromDegrees(-180));

        public static final double kDistToleranceMeters = Units.inchesToMeters(1.75);
/******************************************************************************************************** */

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
//     public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
//     public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);

// The values bellow were taken from pantherbots robot
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(1, 1, 2);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 0.5);


}
