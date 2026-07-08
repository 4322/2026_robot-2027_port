package frc.robot.subsystems.vision.visionObjectDetection;

import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.util.Units;
import frc.robot.constants.Constants;
import java.util.ArrayList;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class VisionObjectDetectionIOPhoton implements VisionObjectDetectionIO {
  private final PhotonCamera photonCamera;

  public VisionObjectDetectionIOPhoton() {
    PhotonCamera.setVersionCheckEnabled(false);
    photonCamera = new PhotonCamera(Constants.VisionObjectDetection.objectCamera);
  }

  @Override
  public void updateInputs(VisionObjectDetectionIOInputsAutoLogged inputs) {
    if (!photonCamera.isConnected()) {
      updateNoNewResultInputs(inputs);
      return;
    }

    final PhotonPipelineResult result = getLatestPipelineResult();
    if (result == null || !result.hasTargets()) {
      updateNoNewResultInputs(inputs);
      return;
    }

    updateHasNewResultInputs(inputs, result);
  }

  private PhotonPipelineResult getLatestPipelineResult() {
    final List<PhotonPipelineResult> unreadResults = photonCamera.getAllUnreadResults();
    return unreadResults.isEmpty() ? null : unreadResults.get(unreadResults.size() - 1);
  }

  private void updateNoNewResultInputs(VisionObjectDetectionIOInputsAutoLogged inputs) {
    inputs.hasTarget = false;
    inputs.visibleObjectRotations = new Rotation3d[0];
  }

  private void updateHasNewResultInputs(
      VisionObjectDetectionIOInputsAutoLogged inputs, PhotonPipelineResult result) {
    final ArrayList<Rotation3d> visibleObjectRotations = new ArrayList<Rotation3d>();
    inputs.hasTarget = false;
    inputs.latestResultTimestamp = result.getTimestampSeconds();

    for (PhotonTrackedTarget currentTarget : result.getTargets()) {
      if (currentTarget.getDetectedObjectClassID() == -1) continue;

      inputs.hasTarget = true;
      visibleObjectRotations.add(extractRotation3d(currentTarget));
    }
    inputs.visibleObjectRotations = toArray(visibleObjectRotations);
  }

  private Rotation3d[] toArray(ArrayList<Rotation3d> arrayList) {
    Rotation3d[] array = new Rotation3d[arrayList.size()];
    for (int i = 0; i < arrayList.size(); i++) {
      array[i] = arrayList.get(i);
    }
    return array;
  }

  private Rotation3d extractRotation3d(PhotonTrackedTarget target) {
    return new Rotation3d(
        0, Units.degreesToRadians(-target.getPitch()), Units.degreesToRadians(-target.getYaw()));
  }
}
