package frc.robot.subsystems.vision.visionGlobalPose;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionGlobalPoseIO {
  @AutoLog
  public static class VisionGlobalPoseIOInputs {
    public boolean connected = false;
    public boolean unstable = false;
    public TargetObservation latestTargetObservation =
        new TargetObservation(Rotation2d.kZero, Rotation2d.kZero);
    public GlobalPoseObservation[] globalPoseObservations = new GlobalPoseObservation[0];
  }

  /** Represents the angle to a simple target, not used for pose estimation. */
  public static record TargetObservation(Rotation2d tx, Rotation2d ty) {}

  /** Represents a robot pose sample used for pose estimation. */
  public static record GlobalPoseObservation(
      double timestamp,
      Pose3d pose,
      Pose3d altPose,
      double ambiguity,
      int tagCount,
      double averageTagDistance,
      double averageTagDistanceAlt,
      boolean useMultiTag) {}

  public default void updateInputs(VisionGlobalPoseIOInputs inputs) {}

  // For sim use to jump the pose
  public default void setRobotPose(Pose2d robotPose) {}
}
