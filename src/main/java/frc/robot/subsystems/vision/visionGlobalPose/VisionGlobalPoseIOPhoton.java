package frc.robot.subsystems.vision.visionGlobalPose;

import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.system.Timer;
import frc.robot.constants.FieldConstants;
import java.util.LinkedList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.MultiTargetPNPResult;
import org.photonvision.targeting.PhotonPipelineResult;

public class VisionGlobalPoseIOPhoton implements VisionGlobalPoseIO {

    protected final PhotonCamera camera;

    protected final Transform3d robotToCamera;

    private Timer connectTime = new Timer();

    private Timer disconnectTime = new Timer();

    /**
     * Creates a new VisionIOPhotonVision.
     *
     * @param name The configured name of the camera.
     * @param rotationSupplier The 3D position of the camera relative to the robot.
     */
    public VisionGlobalPoseIOPhoton(String name, Transform3d robotToCamera) {
        camera = new PhotonCamera(name);
        this.robotToCamera = robotToCamera;
    }

    @Override
    public void updateInputs(VisionGlobalPoseIOInputs inputs) {
        // Detect connection instability
        boolean isNowConnected = camera.isConnected();
        if (isNowConnected && !inputs.connected) {
            connectTime.restart();
        } else if (!isNowConnected && inputs.connected) {
            disconnectTime.restart();
            inputs.unstable = true;
        }
        if (connectTime.hasElapsed(3) || disconnectTime.hasElapsed(3)) {
            inputs.unstable = false;
        }
        inputs.connected = isNowConnected;
        // Read new camera observations
        List<GlobalPoseObservation> globalPoseObservations = new LinkedList<>();
        for (PhotonPipelineResult result : camera.getAllUnreadResults()) {
            // Update latest target observation
            if (result.hasTargets()) {
                inputs.latestTargetObservation = new TargetObservation(Rotation2d.fromDegrees(result.getBestTarget().getYaw()), Rotation2d.fromDegrees(result.getBestTarget().getPitch()));
            } else {
                inputs.latestTargetObservation = new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
            }
            if (result.multitagResult.isPresent()) {
                // Multitag result
                MultiTargetPNPResult multitagResult = result.multitagResult.get();
                // Calculate robot pose
                Transform3d fieldToCamera = multitagResult.estimatedPose.best;
                Transform3d fieldToCameraAlt = multitagResult.estimatedPose.alt;
                Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                Transform3d fieldToRobotAlt = fieldToCameraAlt.plus(robotToCamera.inverse());
                Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
                Pose3d altPose3d = new Pose3d(fieldToRobotAlt.getTranslation(), fieldToRobotAlt.getRotation());
                // Calculate average tag distance
                double totalTagDistance = 0.0;
                double totalTagDistanceAlt = 0.0;
                for (var target : result.targets) {
                    totalTagDistance += target.bestCameraToTarget.getTranslation().getNorm();
                    totalTagDistanceAlt += target.altCameraToTarget.getTranslation().getNorm();
                }
                // Add observation
                globalPoseObservations.add(new GlobalPoseObservation(result.getTimestampSeconds(), robotPose, altPose3d, multitagResult.estimatedPose.ambiguity, multitagResult.fiducialIDsUsed.size(), totalTagDistance / result.targets.size(), totalTagDistanceAlt / result.targets.size(), true));
            } else if (!result.targets.isEmpty()) {
                // Single tag result
                var target = result.targets.get(0);
                // Calculate robot pose
                var tagPose = FieldConstants.aprilTagFieldLayout.getTagPose(target.fiducialId);
                if (tagPose.isPresent()) {
                    Transform3d fieldToTarget = new Transform3d(tagPose.get().getTranslation(), tagPose.get().getRotation());
                    Transform3d cameraToTarget = target.bestCameraToTarget;
                    Transform3d cameraToTargetAlt = target.altCameraToTarget;
                    Transform3d fieldToCamera = fieldToTarget.plus(cameraToTarget.inverse());
                    Transform3d fieldToCameraAlt = fieldToTarget.plus(cameraToTargetAlt.inverse());
                    Transform3d fieldToRobot = fieldToCamera.plus(robotToCamera.inverse());
                    Transform3d fieldToRobotAlt = fieldToCameraAlt.plus(robotToCamera.inverse());
                    Pose3d robotPose = new Pose3d(fieldToRobot.getTranslation(), fieldToRobot.getRotation());
                    Pose3d robotPoseAlt = new Pose3d(fieldToRobotAlt.getTranslation(), fieldToRobotAlt.getRotation());
                    // Add observation
                    globalPoseObservations.add(new GlobalPoseObservation(result.getTimestampSeconds(), robotPose, robotPoseAlt, target.poseAmbiguity, 1, cameraToTarget.getTranslation().getNorm(), cameraToTargetAlt.getTranslation().getNorm(), false));
                }
            }
        }
        // Save pose observations to inputs object
        inputs.globalPoseObservations = new GlobalPoseObservation[globalPoseObservations.size()];
        for (int i = 0; i < globalPoseObservations.size(); i++) {
            inputs.globalPoseObservations[i] = globalPoseObservations.get(i);
        }
    }
}
