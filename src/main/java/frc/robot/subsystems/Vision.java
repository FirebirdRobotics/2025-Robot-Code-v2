// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Robot;
import frc.robot.constants.VisionConstants;
import frc.robot.constants.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;

import java.util.List;
import java.util.Optional;
import java.util.Vector;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import dev.doglog.DogLog;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix6.Utils;


public class Vision extends SubsystemBase {
  /** Creates a new Vision. */


  private final PhotonCamera aprilCamRight;
  private final PhotonCamera aprilCamLeft;
  private final PhotonCamera objectCam;

  private final PhotonPoseEstimator photonEstimatorRight;
  private final PhotonPoseEstimator photonEstimatorLeft;
  private Matrix<N3, N1> curStdDevs;
  CommandSwerveDrivetrain drivetrain;
  
  // Simulation
  private PhotonCameraSim cameraSim;
  private VisionSystemSim visionSim;



  public Vision(CommandSwerveDrivetrain m_CommandSwerveDrivetrain) {
    drivetrain = m_CommandSwerveDrivetrain;
    aprilCamRight = new PhotonCamera(VisionConstants.kaprilCamRightName);
    aprilCamLeft = new PhotonCamera(VisionConstants.kaprilCamLeftName);
    objectCam = new PhotonCamera(VisionConstants.kobjectCamName);
    photonEstimatorRight =
          new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToRightAprilCam);
    photonEstimatorLeft =
          new PhotonPoseEstimator(VisionConstants.kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, VisionConstants.kRobotToLeftAprilCam);
    photonEstimatorRight.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
    photonEstimatorLeft.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        // ----- Simulation
        if (Robot.isSimulation()) {
            // Create the vision system simulation which handles cameras and targets on the field.
            visionSim = new VisionSystemSim("main");
            // Add all the AprilTags inside the tag layout as visible targets to this simulated field.
            visionSim.addAprilTags(VisionConstants.kTagLayout);
            // Create simulated camera properties. These can be set to mimic your actual camera.
            var cameraProp = new SimCameraProperties();
            cameraProp.setCalibration(960, 720, Rotation2d.fromDegrees(90));
            cameraProp.setCalibError(0.35, 0.10);
            cameraProp.setFPS(15);
            cameraProp.setAvgLatencyMs(50);
            cameraProp.setLatencyStdDevMs(15);
            // Create a PhotonCameraSim which will update the linked PhotonCamera's values with visible
            // targets.
            cameraSim = new PhotonCameraSim(aprilCamLeft, cameraProp);
            // Add the simulated camera to view the targets on this simulated field.
            visionSim.addCamera(cameraSim, VisionConstants.kRobotToLeftAprilCam);

            cameraSim.enableDrawWireframe(true);
      }
  }

      /**
     * The latest estimated robot pose on the field from vision data. This may be empty. This should
     * only be called once per loop.
     *
     * <p>Also includes updates for the standard deviations, which can (optionally) be retrieved with
     * {@link getEstimationStdDevs}
     *
     * @param cam The PhotonCamera to use
     * @param estimator The PhotonPoseEstimator to use
     *
     * @return An {@link EstimatedRobotPose} with an estimated pose, estimate timestamp, and targets
     *     used for estimation.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonCamera cam, PhotonPoseEstimator estimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : cam.getAllUnreadResults()) {
            visionEst = estimator.update(change);
            updateEstimationStdDevs(visionEst, change.getTargets());
            
            if (Robot.isSimulation()) {
                visionEst.ifPresentOrElse(
                        est ->
                                getSimDebugField()
                                        .getObject("VisionEstimation")
                                        .setPose(est.estimatedPose.toPose2d()),
                        () -> {
                            getSimDebugField().getObject("VisionEstimation").setPoses();
                        });
            }
        }
        return visionEst;
    }

    /**
     * Creates a list of combined pose estimates from both cameras.
     *
     * @return An {@link List<EstimatedRobotPose>} with estimated poses, timestamps, and targets.
     */
    public List<Optional<EstimatedRobotPose>> getEstimatedGlobalPoses() {
      var estPoses = List.of(getEstimatedGlobalPose(aprilCamLeft, photonEstimatorLeft), getEstimatedGlobalPose(aprilCamRight, photonEstimatorRight));
      return estPoses;
    }
    

    /**
     * Returns closest AprilTag
     * 
     * @return A {@link PhotonTrackedTarget} of the closest AprilTag
     */
    public Optional<PhotonTrackedTarget> getClosestAprilTag() {
        Vector<PhotonTrackedTarget> tags = new Vector<PhotonTrackedTarget>();
        var results = List.of(aprilCamLeft.getLatestResult(), aprilCamRight.getLatestResult());
        for(var res : results){
            if(res.hasTargets()){
                tags.add(res.getBestTarget());
            }
        }
        if(tags.size() == 1){
            return Optional.of(tags.get(0));
        }
        else if(!tags.isEmpty()){
            double ambi = tags.get(0).getPoseAmbiguity();
            if(tags.get(1).getPoseAmbiguity() < ambi){
                return Optional.of(tags.get(1));
            }
            else{
                return Optional.of(tags.get(0));
            }
        }
        else{
            return Optional.empty();
        }
    }

public Optional<Pose2d> getClosestAprilPose(){
    Optional<PhotonTrackedTarget> tag = getClosestAprilTag();
    if(tag.isEmpty()){
        return Optional.empty();
    }
    int id = tag.get().fiducialId;
    Optional<Pose3d> pose = VisionConstants.kTagLayout.getTagPose(id);
    if(pose.isPresent() && ((id > 6 && id < 11) || (id > 17 && id < 22))){
        return Optional.of(pose.get().toPose2d());
    }
    else{
        return Optional.empty();
    }
}

public Pose2d getShiftedAprilPose(Pose2d apose, boolean direction){
    double x0 = apose.getX();
    double y0 = apose.getY();
    double theta = apose.getRotation().getRadians();

    double x0p = (x0*Math.cos(theta))-(y0*Math.sin(theta));
    double y0p = (x0*Math.sin(theta))+(y0*Math.cos(theta));

    if(direction){
        // Right
        y0p += VisionConstants.kTagToBranch;
    }
    else{
        // Left
        y0p -= VisionConstants.kTagToBranch;
    }

    x0p += VisionConstants.kPersonalSpace;

    x0 = (x0p*Math.cos(-theta))-(y0p*Math.sin(-theta));
    y0 = (x0p*Math.sin(-theta))+(y0p*Math.cos(-theta));

    Pose2d shifted = new Pose2d(x0, y0, apose.getRotation());

    return shifted;

}


public Optional<Pose2d> getClosestBranchPose(boolean direction){
    Optional<Pose2d> pose = getClosestAprilPose();
    if(pose.isPresent()){
        return Optional.of(getShiftedAprilPose(pose.get(), direction));
    }
    
    return Optional.empty();
}

public Optional<Transform3d> robotToTag(Pose2d robot2d, PhotonTrackedTarget target){
    int id = target.getFiducialId();
    Rotation3d rot = new Rotation3d(robot2d.getRotation());
    Pose3d robot = new Pose3d(robot2d.getX(), robot2d.getY(), 0, rot);
    if(id == -1){
        return Optional.empty();
    }

    Optional<Pose3d> aprilPose = VisionConstants.kTagLayout.getTagPose(id);

    if(aprilPose.isPresent()){
        double x0 = aprilPose.get().getX();
        double y0 = aprilPose.get().getY();
        double x1 = robot.getX();
        double y1 = robot.getY();
        double theta = -(aprilPose.get().getRotation().getAngle()+Units.degreesToRadians(180));

        double x0p = (x0*Math.cos(theta))-(y0*Math.sin(theta));
        double y0p = (x0*Math.sin(theta))+(y0*Math.cos(theta));
        double x1p = (x1*Math.cos(theta))-(y1*Math.sin(theta));
        double y1p = (x1*Math.sin(theta))+(y1*Math.cos(theta));

        Rotation2d norotate = new Rotation2d(0);
        Pose2d apose2 = new Pose2d(x0p, y0p, norotate);
        Pose2d rpose2 = new Pose2d(x1p, y1p, norotate);

        Pose3d apose = new Pose3d(apose2);
        Pose3d rpose = new Pose3d(rpose2);

        Transform3d transf = new Transform3d(apose, rpose);
        return Optional.of(transf);
    }
    else{
        return Optional.empty();
    }

}

    /**
     * Calculates new standard deviations This algorithm is a heuristic that creates dynamic standard
     * deviations based on number of tags, estimation strategy, and distance from the tags.
     *
     * @param estimatedPose The estimated pose to guess standard deviations for.
     * @param targets All targets in this camera frame
     */
    private void updateEstimationStdDevs(
            Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            curStdDevs = VisionConstants.kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            var estStdDevs = VisionConstants.kSingleTagStdDevs;
            int numTags = 0;
            double avgDist = 0;

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimatorLeft.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                curStdDevs = VisionConstants.kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                avgDist /= numTags;
                // Decrease std devs if multiple targets are visible
                if (numTags > 1) estStdDevs = VisionConstants.kMultiTagStdDevs;
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4)
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));
                curStdDevs = estStdDevs;
            }
        }
    }

    /**
     * Returns the latest standard deviations of the estimated pose from {@link
     * #getEstimatedGlobalPose()}, for use with {@link
     * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}. This should
     * only be used when there are targets visible.
     */
    public Matrix<N3, N1> getEstimationStdDevs() {
      return curStdDevs;
  }

    
  
  // ----- Simulation
  public void simulationPeriodic(Pose2d robotSimPose) {
      visionSim.update(robotSimPose);
  }

    /** Reset pose history of the robot in the vision system simulation. */
    public void resetSimPose(Pose2d pose) {
      if (Robot.isSimulation()) visionSim.resetRobotPose(pose);
  }

    /** A Field2d for visualizing our robot and objects on the field. */
    public Field2d getSimDebugField() {
        if (!Robot.isSimulation()) return null;
        return visionSim.getDebugField();
    }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // drivetrain.addVisionMeasurement(camera.getLatestResult(), );
        List<Optional<EstimatedRobotPose>> poses = this.getEstimatedGlobalPoses();
        for( Optional<EstimatedRobotPose> visionEst : poses ){
            visionEst.ifPresent(
                        est -> {
                            // Change our trust in the measurement based on the tags we can see
                            var estStdDevs = this.getEstimationStdDevs();

                            drivetrain.addVisionMeasurement(
                                   est.estimatedPose.toPose2d(), Utils.fpgaToCurrentTime(Timer.getFPGATimestamp()), estStdDevs);
                            DogLog.log("Vision Pose", est.estimatedPose.toPose2d());
                        }
                        );
        }

        // Give a visual cue for alignment to reef
        var i = getClosestAprilTag();
        if(i.isPresent()){
            var t = robotToTag(drivetrain.getState().Pose, i.get());
            // var t = robotToTag(est.estimatedPose.toPose2d(), i.get());
            if(t.isPresent()){
                DogLog.log("Alignment Values", t.get());
                if(Math.abs(t.get().getY()) <= VisionConstants.kTolerance){
                    DogLog.log("Aligned", true);
                }else{
                    DogLog.log("Aligned", false);
                }
            }
        }
        else{
            DogLog.log("Aligned", false);
        }
        
    }


    @Override
    public void simulationPeriodic() {
        // Update drivetrain simulation
        drivetrain.simulationPeriodic();

        // Update camera simulation
        this.simulationPeriodic(drivetrain.getState().Pose);
        var debugField = this.getSimDebugField();
        debugField.getObject("EstimatedRobot").setPose(drivetrain.getState().Pose);
        debugField.getObject("EstimatedRobotModules").setPoses(drivetrain.getState().Pose);




    }

    // Creates a PIDController with gains kP, kI, and kD
    public final double kXP = 0;
    public final double kXI = 0;
    public final double kXD = 0;

    public final double kYP = 0;
    public final double kYI = 0;
    public final double kYD = 0;

    public final double kRotationP = 0;
    public final double kRotationI = 0;
    public final double kRotationD = 0;


//  // tx ty are angles 
// // field centric facing angle
//     PIDController drivetrainXDumbPID = new PIDController(kXP, kXI, kXD);
//     PIDController drivetrainYDumbPID = new PIDController(kYP, kYI, kYD);
//     PIDController drivetrainRotationDumbPID = new PIDController(kRotationP, kRotationI, kRotationD);

//     private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
//     private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

//     public Command dumbPIDAlignToAiprilTag() {
    
//         drivetrain.applyRequest(() ->
//         drive.withVelocityX(drivetrainXDumbPID.calculate(MaxAngularRate) * MaxSpeed) // Drive forward with negative Y (forward)
//             .withVelocityY( * MaxSpeed) // Drive left with negative X (left)
//             .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
//     )
// }
}
