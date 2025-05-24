package frc.robot.subsystems;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//import edu.wpi.first.wpilibj.simulation.VisionSystemSim;

public class PhotonVisionSubsystem extends SubsystemBase {

    private final PhotonCamera camera = new PhotonCamera("maincam");
    private final AprilTagFieldLayout aprilTagFieldLayout;
    private final Transform3d robotToCamera = new Transform3d(
        new Translation3d(0.35, 0.0, 0.15),  // X, Y, Z offsets in meters
        new Rotation3d(0.0, 0.0, 0.0)        // No rotation
    );

    private final PhotonPoseEstimator photonPoseEstimator;

    // Nullable visionSim, only initialized in simulation
    //private final VisionSystemSim visionSim;

    public PhotonVisionSubsystem() {
        aprilTagFieldLayout = AprilTagFields.k2025ReefscapeAndyMark.loadAprilTagLayoutField();

        photonPoseEstimator = new PhotonPoseEstimator(
            aprilTagFieldLayout,
            PoseStrategy.CLOSEST_TO_REFERENCE_POSE,
            robotToCamera
        );

        camera.setDriverMode(true);
        camera.setPipelineIndex(2);

        // Only instantiate visionSim if running in simulation
        if (RobotBase.isSimulation()) {
            //visionSim = new VisionSystemSim("main");
        } else {
            //visionSim = null;
        }
    }

    public void processVision() {
        var result = camera.getLatestResult();
        if (!result.hasTargets()) return;

        PhotonTrackedTarget target = result.getBestTarget();

        camera.takeInputSnapshot();
        camera.takeOutputSnapshot();

        if (aprilTagFieldLayout.getTagPose(target.getFiducialId()).isPresent()) {
            Pose3d tagPose = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();

            Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(
                target.getBestCameraToTarget(),
                tagPose,
                robotToCamera
            );

            Pose2d robotPose2d = robotPose.toPose2d();
            Pose2d tagPose2d = tagPose.toPose2d();

            double distanceMeters = robotPose2d.getTranslation().getDistance(tagPose2d.getTranslation());

            Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
                distanceMeters,
                Rotation2d.fromDegrees(-target.getYaw())
            );

            Rotation2d targetYaw = tagPose2d.getRotation().minus(robotPose2d.getRotation());

            System.out.println("Distance: " + distanceMeters + " m");
            System.out.println("Target Yaw: " + targetYaw.getDegrees() + " degrees");
        }
    }

    /**
     * Get the estimated global robot pose given a prior estimate.
     * @param prevEstimatedRobotPose The previous estimated pose of the robot.
     * @return Optional EstimatedRobotPose with the latest pose estimate from PhotonVision.
     */
    public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
        return photonPoseEstimator.update(null);
    }

    /**
     * Accessor for the vision simulation instance.
     * Returns null if not in simulation.
     */
    //public VisionSystemSim getVisionSim() {
        //return visionSim;
    }
//}
