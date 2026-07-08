package frc.robot.subsystems.vision.visionGlobalPose;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Transform3d;
import frc.robot.constants.FieldConstants;
import java.util.function.Supplier;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

/** IO implementation for physics sim using PhotonVision simulator. */
public class VisionGlobalPoseIOSim extends VisionGlobalPoseIOPhoton {
  private static VisionSystemSim visionSim;

  private final Supplier<Pose2d> poseSupplier;
  private final PhotonCameraSim cameraSim;

  /**
   * Creates a new VisionIOPhotonVisionSim.
   *
   * @param name The name of the camera.
   * @param poseSupplier Supplier for the robot pose to use in simulation.
   */
  public VisionGlobalPoseIOSim(
      String name, Supplier<Pose2d> poseSupplier, Transform3d robotToCamera) {
    super(name, robotToCamera);
    this.poseSupplier = poseSupplier;

    // Initialize vision sim
    if (visionSim == null) {
      visionSim = new VisionSystemSim("main");
      visionSim.addAprilTags(FieldConstants.aprilTagFieldLayout);
    }

    // Add sim camera
    var cameraProperties = new SimCameraProperties();
    cameraSim = new PhotonCameraSim(camera, cameraProperties, FieldConstants.aprilTagFieldLayout);
    visionSim.addCamera(cameraSim, robotToCamera);
  }

  @Override
  public void updateInputs(VisionGlobalPoseIOInputs inputs) {
    visionSim.update(poseSupplier.get());
    super.updateInputs(inputs);
  }

  @Override
  public void setRobotPose(Pose2d robotPose) {
    visionSim.resetRobotPose(robotPose);
  }
}
