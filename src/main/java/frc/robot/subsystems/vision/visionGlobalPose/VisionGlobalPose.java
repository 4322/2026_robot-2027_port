package frc.robot.subsystems.vision.visionGlobalPose;

import org.wpilib.math.linalg.Matrix;
import org.wpilib.math.linalg.VecBuilder;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.numbers.N1;
import org.wpilib.math.numbers.N3;
import org.wpilib.telemetry.Telemetry;
import org.wpilib.telemetry.TelemetryTable;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPoseIO.GlobalPoseObservation;
import frc.robot.util.LoggedTunableNumber;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class VisionGlobalPose extends SubsystemBase {
  private final VisionConsumer consumer;
  private final VisionGlobalPoseIO[] io;
  private final VisionGlobalPoseIOInputsAutoLogged[] inputs;
  private final Drive drive;
  private static final LoggedTunableNumber baseStdDev =
      new LoggedTunableNumber("GlobalPose/baseStdDev", Constants.VisionGlobalPose.stdDevBaseline);
  private static final LoggedTunableNumber singleTagStdDev =
      new LoggedTunableNumber(
          "GlobalPose/singleTagStdDev", Constants.VisionGlobalPose.singleTagStdDevAdjuster);
  private static final LoggedTunableNumber maxAvgTagDistance =
      new LoggedTunableNumber(
          "GlobalPose/maxAvgTagDistance", Constants.VisionGlobalPose.maxAvgTagDistance);
  private final TelemetryTable visionTelemetry = Telemetry.getTable("Vision");

  public VisionGlobalPose(Drive drive, VisionGlobalPoseIO... io) {
    this.drive = drive;
    this.consumer = drive::addVisionMeasurement;
    this.io = io;

    // Initialize inputs
    this.inputs = new VisionGlobalPoseIOInputsAutoLogged[io.length];
    for (int i = 0; i < inputs.length; i++) {
      inputs[i] = new VisionGlobalPoseIOInputsAutoLogged();
    }
  }

  /**
   * Returns the X angle to the best target, which can be used for simple servoing with vision.
   *
   * @param cameraIndex The index of the camera to use.
   */
  public Rotation2d getTargetX(int cameraIndex) {
    return inputs[cameraIndex].latestTargetObservation.tx();
  }

  List<Pose2d> allRobotPoses = new ArrayList<>();
  List<Pose2d> allRobotPosesAccepted = new ArrayList<>();
  List<Pose2d> allRobotPosesRejected = new ArrayList<>();

  List<Pose2d> robotPoses = new ArrayList<>();
  List<Pose2d> robotPosesAccepted = new ArrayList<>();
  List<Pose2d> robotPosesRejected = new ArrayList<>();

  @Override
  public void periodic() {
    int unstableCameras = 0;
    for (int i = 0; i < io.length; i++) {
      io[i].updateInputs(inputs[i]);
      Logger.processInputs("Vision/Camera" + Integer.toString(i), inputs[i]);
      if (inputs[i].unstable) {
        unstableCameras++;
      }
    }

    if (unstableCameras >= 2) {
      visionTelemetry.log("Vision/Vision Stable", Constants.NetworkTables.red.toHexString());
      Logger.recordOutput("Vision/Stable", "No");
      Robot.stabilize();
    } else {
      visionTelemetry.log("Vision/Vision Stable", Constants.NetworkTables.green.toHexString());
      Logger.recordOutput("Vision/Stable", "Yes");
    }

    // Initialize logging values
    allRobotPoses.clear();
    allRobotPosesAccepted.clear();
    allRobotPosesRejected.clear();

    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      // Initialize logging values
      robotPoses.clear();
      robotPosesAccepted.clear();
      robotPosesRejected.clear();

      for (GlobalPoseObservation observation : inputs[cameraIndex].globalPoseObservations) {
        Pose3d disambiguatedRobotPose; // Robot pose chosen after disambiguation
        double avgTagDistance = observation.averageTagDistance();

        // Disambiguate to select a single robot pose
        if (observation.useMultiTag()) {
          disambiguatedRobotPose = observation.pose();
        } else {
          if (observation.ambiguity() < Constants.VisionGlobalPose.maxAmbiguity) {
            disambiguatedRobotPose = observation.pose();
          } else if (Math.abs(
                  observation
                      .pose()
                      .toPose2d()
                      .getRotation()
                      .minus(drive.getRotation())
                      .getRadians())
              < Math.abs(
                  observation
                      .altPose()
                      .toPose2d()
                      .getRotation()
                      .minus(drive.getRotation())
                      .getRadians())) {
            disambiguatedRobotPose = observation.pose();
          } else {
            disambiguatedRobotPose = observation.altPose();
            avgTagDistance = observation.averageTagDistanceAlt();
          }
        }

        // Check whether to reject pose
        boolean rejectPose =
            observation.tagCount() == 0 // Must have at least one tag
                || (observation.tagCount() == 1
                    && observation.ambiguity()
                        > Constants.VisionGlobalPose.maxAmbiguity) // Cannot be high ambiguity
                || Math.abs(disambiguatedRobotPose.getZ())
                    > Constants.VisionGlobalPose.maxZError // Must have realistic Z coordinate

                // Must be within the field boundaries
                || disambiguatedRobotPose.getX() < -0.5
                || disambiguatedRobotPose.getX() > FieldConstants.fieldLength + 0.5
                || disambiguatedRobotPose.getY() < -0.5
                || disambiguatedRobotPose.getY() > FieldConstants.fieldWidth + 0.5
                || avgTagDistance > maxAvgTagDistance.get();

        // Add pose to log
        if (Constants.VisionGlobalPose.enableVerbosePoseLogging) {
          robotPoses.add(disambiguatedRobotPose.toPose2d());
          if (rejectPose) {
            robotPosesRejected.add(disambiguatedRobotPose.toPose2d());
          } else {
            robotPosesAccepted.add(disambiguatedRobotPose.toPose2d());
          }
        }

        // Skip if rejected
        if (rejectPose) {
          continue;
        }

        // Calculate standard deviations
        double xyStdDev = 0.0;
        double thetaStdDev = 0.0;

        if (observation.useMultiTag()) {
          xyStdDev = (avgTagDistance * avgTagDistance) / observation.tagCount();
          thetaStdDev = (avgTagDistance * avgTagDistance) / observation.tagCount();

          consumer.accept(
              disambiguatedRobotPose.toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(
                  baseStdDev.get() * xyStdDev,
                  baseStdDev.get() * xyStdDev,
                  baseStdDev.get() * thetaStdDev));
          if (Constants.VisionGlobalPose.enableVerbosePoseLogging) {
            Logger.recordOutput(
                "VisionGlobalPose/Camera" + Integer.toString(cameraIndex) + "/StdDev/XY",
                baseStdDev.get() * xyStdDev);
            Logger.recordOutput(
                "VisionGlobalPose/Camera" + Integer.toString(cameraIndex) + "/StdDev/Theta",
                baseStdDev.get() * thetaStdDev);
          }
        } else {
          xyStdDev = avgTagDistance * avgTagDistance;
          thetaStdDev = avgTagDistance * avgTagDistance;

          consumer.accept(
              disambiguatedRobotPose.toPose2d(),
              observation.timestamp(),
              VecBuilder.fill(
                  baseStdDev.get() * xyStdDev * singleTagStdDev.get(),
                  baseStdDev.get() * xyStdDev * singleTagStdDev.get(),
                  baseStdDev.get() * thetaStdDev * singleTagStdDev.get()));

          if (Constants.VisionGlobalPose.enableVerbosePoseLogging) {
            Logger.recordOutput(
                "VisionGlobalPose/Camera" + Integer.toString(cameraIndex) + "/StdDev/XY",
                baseStdDev.get() * xyStdDev * singleTagStdDev.get());
            Logger.recordOutput(
                "VisionGlobalPose/Camera" + Integer.toString(cameraIndex) + "/StdDev/Theta",
                baseStdDev.get() * thetaStdDev * singleTagStdDev.get());
          }
        }
      }

      // Log camera data
      if (Constants.VisionGlobalPose.enableVerbosePoseLogging) {
        Logger.recordOutput(
            "VisionGlobalPose/Camera" + Integer.toString(cameraIndex) + "/RobotPosesAccepted",
            robotPosesAccepted.stream().toArray(Pose2d[]::new));
        Logger.recordOutput(
            "VisionGlobalPose/Camera" + Integer.toString(cameraIndex) + "/RobotPosesRejected",
            robotPosesRejected.stream().toArray(Pose2d[]::new));
        allRobotPoses.addAll(robotPoses);
        allRobotPosesAccepted.addAll(robotPosesAccepted);
        allRobotPosesRejected.addAll(robotPosesRejected);
      }
    }

    // Log summary data
    if (Constants.VisionGlobalPose.enableVerbosePoseLogging) {
      Logger.recordOutput(
          "VisionGlobalPose/Summary/RobotPoses", allRobotPoses.stream().toArray(Pose2d[]::new));
      Logger.recordOutput(
          "VisionGlobalPose/Summary/RobotPosesAccepted",
          allRobotPosesAccepted.stream().toArray(Pose2d[]::new));
      Logger.recordOutput(
          "VisionGlobalPose/Summary/RobotPosesRejected",
          allRobotPosesRejected.stream().toArray(Pose2d[]::new));
    }
  }

  @FunctionalInterface
  public static interface VisionConsumer {
    public void accept(
        Pose2d visionRobotPoseMeters,
        double timestampSeconds,
        Matrix<N3, N1> visionMeasurementStdDevs);
  }

  public void setRobotPose(Pose2d robotPose) {
    // Loop over cameras
    for (int cameraIndex = 0; cameraIndex < io.length; cameraIndex++) {
      io[cameraIndex].setRobotPose(robotPose);
    }
  }
}
