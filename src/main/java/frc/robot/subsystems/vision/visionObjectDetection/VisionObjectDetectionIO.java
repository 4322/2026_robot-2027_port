package frc.robot.subsystems.vision.visionObjectDetection;

import org.wpilib.math.geometry.Rotation3d;
import org.littletonrobotics.junction.AutoLog;

public interface VisionObjectDetectionIO {

  @AutoLog
  public static class VisionObjectDetectionIOInputs {
    public boolean hasTarget;
    public Rotation3d[] visibleObjectRotations;

    public double latestResultTimestamp;
  }

  public default void updateInputs(VisionObjectDetectionIOInputsAutoLogged inputs) {}
}
