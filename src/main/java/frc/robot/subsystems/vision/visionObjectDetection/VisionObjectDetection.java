package frc.robot.subsystems.vision.visionObjectDetection;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.command2.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.VisionObjectDetection.ObjectDetectionTarget;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import org.littletonrobotics.junction.Logger;

public class VisionObjectDetection extends SubsystemBase {
  private final VisionObjectDetectionIOInputsAutoLogged inputs =
      new VisionObjectDetectionIOInputsAutoLogged();
  private final VisionObjectDetectionIO io;
  private final Transform3d robotCenterToCamera;
  private final Drive drive;
  private Translation2d bestFuelPosition;

  public enum ObjectDetectionType {
    COLOR,
    OBJECT
  }

  public VisionObjectDetection(Drive drive, VisionObjectDetectionIO io) {
    this.drive = drive;
    this.io = io;
    this.robotCenterToCamera = Constants.VisionObjectDetection.robotCenterToCamera;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Vision/ObjectDetectionCamera", inputs);

    if (Constants.VisionObjectDetection.enableObjectDetectionDebug) {
      bestFuelPosition = calculateBestObjectPositionOnField(true);

      if (bestFuelPosition != null) {
        Logger.recordOutput(
            "VisionObjectDetection/bestFuelFieldPos",
            new Pose2d(bestFuelPosition, Rotation2d.ZERO));
      }
    }
  }

  public Translation2d calculateBestObjectPositionOnField(boolean sameZone) {
    final Translation2d[] targetObjectsTranslation = getObjectPositionsOnField();
    final Translation2d currentRobotTranslation = drive.getRobotPose().getTranslation();

    if (targetObjectsTranslation.length == 0) return null;

    Translation2d bestObjectTranslation = null;
    for (int i = 0; i < targetObjectsTranslation.length; i++) {
      final Translation2d currentObjectTranslation = targetObjectsTranslation[i];
      final double bestObjectDifference =
          currentRobotTranslation.getDistance(bestObjectTranslation);
      final double currentObjectDifference =
          currentRobotTranslation.getDistance(currentObjectTranslation);
      if (currentObjectDifference < bestObjectDifference
          && (!sameZone
              || AreaManager.isSameCompleteZone(
                  new Pose2d(currentObjectTranslation, Rotation2d.ZERO), drive.getRobotPose()))) {
        bestObjectTranslation = currentObjectTranslation;
      }
    }
    return bestObjectTranslation;
  }

  public boolean hasTargets() {
    return inputs.hasTarget;
  }

  public Translation2d[] getObjectPositionsOnField() {
    final Rotation3d[] visibleObjectsRotations = getTargetObjectsRotations();
    final Translation2d[] objectsPositionsOnField =
        new Translation2d[visibleObjectsRotations.length];

    for (int i = 0; i < visibleObjectsRotations.length; i++) {
      objectsPositionsOnField[i] = calculateObjectPositionFromRotation(visibleObjectsRotations[i]);
      Logger.recordOutput(
          "VisionObjectDetection/VisibleLemons",
          new Pose2d(objectsPositionsOnField[i], Rotation2d.ZERO));
    }

    return objectsPositionsOnField;
  }

  public Rotation3d[] getTargetObjectsRotations() {
    return inputs.visibleObjectRotations;
  }

  /**
   * Calculates the position of the object on the field from the 3D rotation of the object relative
   * to the camera. This assumes the object is on the ground. Once it is known that the object is on
   * the ground, one can simply find the transform from the camera to the ground and apply it to the
   * object's rotation.
   *
   * @param objectRotation the object's 3D rotation relative to the camera
   * @return the object's 2D position on the field (z is assumed to be 0)
   */
  private Translation2d calculateObjectPositionFromRotation(Rotation3d objectRotation) {
    final Pose2d robotPoseAtResultTimestamp =
        drive.getPoseAtTimestamp(inputs.latestResultTimestamp);
    if (robotPoseAtResultTimestamp == null) return new Translation2d();
    final Pose3d cameraPose = new Pose3d(robotPoseAtResultTimestamp).plus(robotCenterToCamera);
    final Pose3d objectRotationStart = cameraPose.plus(new Transform3d(0, 0, 0, objectRotation));

    final double cameraZ = cameraPose.getTranslation().getZ();
    final double objectPitchSin = Math.sin(objectRotationStart.getRotation().getY());
    final double xTransform = cameraZ / objectPitchSin;
    final Transform3d objectRotationStartToGround =
        new Transform3d(xTransform, 0, 0, new Rotation3d());

    return objectRotationStart
        .transformBy(objectRotationStartToGround)
        .getTranslation()
        .toTranslation2d();
  }

  // Computes average position of visible objects on field
  public Translation2d getCentroidOfVisibleObjects(boolean sameZone) {
    final Translation2d[] objectPositions = getObjectPositionsOnField();

    if (objectPositions.length == 0) return null;

    double sumX = 0.0;
    double sumY = 0.0;
    int count = 0;

    for (Translation2d objectPosition : objectPositions) {
      if (sameZone
          && !AreaManager.isSameCompleteZone(
              new Pose2d(objectPosition, Rotation2d.ZERO), drive.getRobotPose())) {
        continue;
      }
      sumX += objectPosition.getX();
      sumY += objectPosition.getY();
      count++;
    }

    if (count == 0) return null;

    Translation2d centroid = new Translation2d(sumX / count, sumY / count);
    Logger.recordOutput(
        "VisionObjectDetection/TargetCentroid", new Pose2d(centroid, Rotation2d.ZERO));
    if ((!AreaManager.isSameCompleteZone(
            new Pose2d(centroid, Rotation2d.ZERO), drive.getRobotPose()))
        && sameZone) {
      return null;
    } else {
      return centroid;
    }
  }

  // Attempts to get average fuel position, but if that fails, returns closest fuel
  public Translation2d getBestFuelPose(boolean sameZone) {
    if (Constants.VisionObjectDetection.mode == ObjectDetectionTarget.CENTROID) {
      final Translation2d bestFuelPosition = getCentroidOfVisibleObjects(sameZone);
      if (bestFuelPosition == null) {
        return calculateBestObjectPositionOnField(sameZone);
      }
      return bestFuelPosition;
    } else if (Constants.VisionObjectDetection.mode == ObjectDetectionTarget.CLOSEST) {
      return calculateBestObjectPositionOnField(sameZone);
    } else {
      return null;
    }
  }
}
