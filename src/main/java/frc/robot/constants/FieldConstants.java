package frc.robot.constants;

import org.wpilib.fields.Field;
import org.wpilib.fields.FieldTag;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Quaternion;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
import frc.robot.util.Region2d;
import java.util.List;

public class FieldConstants {
  // Point A should be bottom right
  // Point B should be top left
  // All are relative to the blue alliance station
  // Using Wpilib coordinates
  public static double stopShootingZoneScale = 1.3;

  public static double fieldLength = Units.inchesToMeters(651.22);
  public static double fieldWidth = Units.inchesToMeters(317.69);
  public static double edgeOfHubNeutral = 47.00;
  public static double trenchZoneArea = 22.20;

  public static double bumpLength = Units.inchesToMeters(73.00);

  // Very small tolerance to cover the actual trench bar
  public static double stopShootingZoneBuffer = Units.inchesToMeters(2.0);

  public static double centerLineX = fieldLength / 2;
  public static double centerLineY = fieldWidth / 2;
  public static double blueLineX =
      Units.inchesToMeters(158.6); // The edge towards the driver station of the hub
  public static double redLineX =
      fieldLength
          - Units.inchesToMeters(
              158.6); // The edge towards the driver station of the hub but red side
  public static double hubOffset =
      Units.inchesToMeters(24.0); // Distance between the blue/red line and the respective hub
  public static final double fieldEdgeTolerance =
      Units.inchesToMeters(
          48); // Entire length of side of hub to towards the driverstation to the neutral zone
  // Added to still detect robot with minor odometry or vision discrepancy

  // All left/right designations are relative to blue alliance station
  public static class Blue {
    public static Translation2d hubTranslation =
        new Translation2d(blueLineX + hubOffset, centerLineY); // Middle of the hub
    public static Region2d allianceZone =
        new Region2d(
            new Translation2d(-fieldEdgeTolerance, -fieldEdgeTolerance),
            new Translation2d(blueLineX + hubOffset, fieldWidth + fieldEdgeTolerance),
            "blue/allianceZone");
    public static Region2d rightAllianceZone =
        new Region2d(
            new Translation2d(-fieldEdgeTolerance, -fieldEdgeTolerance),
            new Translation2d(blueLineX + hubOffset, centerLineY),
            "blue/rightAllianceZone");
    public static Region2d leftAllianceZone =
        new Region2d(
            new Translation2d(-fieldEdgeTolerance, centerLineY),
            new Translation2d(blueLineX + hubOffset, fieldWidth + fieldEdgeTolerance),
            "blue/leftAllianceZone");

    // Covers left blue trench (only neutral zone side)
    // For use when we are blue alliance and want to be able to shoot in the trench
    public static Region2d stopShootLeftNeutral =
        new Region2d(
            new Translation2d(
                blueLineX + hubOffset - stopShootingZoneBuffer,
                centerLineY + Units.inchesToMeters(133.47 - 24.97)),
            new Translation2d(
                blueLineX + hubOffset + stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                fieldWidth + fieldEdgeTolerance),
            "blue/stopShootLeftNeutral");

    // Covers right blue trench (only neutral zone side)
    // For use when we are blue alliance and want to be able to shoot in the trench
    public static Region2d stopShootRightNeutral =
        new Region2d(
            new Translation2d(blueLineX + hubOffset - stopShootingZoneBuffer, -fieldEdgeTolerance),
            new Translation2d(
                blueLineX + hubOffset + stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                Units.inchesToMeters(50.59)),
            "blue/stopShootRightNeutral");

    // Covers the left blue trench (both neutral and alliance zone sides)
    // For use when we are red alliance and want to stop shooting anywhere near the trench
    public static Region2d stopShootLeftFull =
        new Region2d(
            new Translation2d(
                blueLineX + hubOffset - stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                centerLineY + Units.inchesToMeters(133.47 - 24.97)),
            new Translation2d(
                blueLineX + hubOffset + stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                fieldWidth + fieldEdgeTolerance),
            "blue/stopShootLeftFull");

    // Covers the right blue trench (both neutral and alliance zone sides)
    // For use when we are red alliance and want to stop shooting anywhere near the trench
    public static Region2d stopShootRightFull =
        new Region2d(
            new Translation2d(
                blueLineX + hubOffset - stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                -fieldEdgeTolerance),
            new Translation2d(
                blueLineX + hubOffset + stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                Units.inchesToMeters(50.59)),
            "blue/stopShootRightFull");

    public static Region2d frontOfHub =
        new Region2d(
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20 + 40),
                centerLineY - Units.inchesToMeters(58.41 / 2)),
            new Translation2d(
                blueLineX - Units.inchesToMeters(22.20),
                centerLineY + Units.inchesToMeters(58.41 / 2)),
            "blue/frontOfHub");

    // These zones are based on which side they're passing to, facing the blue driver station
    // Only cover neutral zone

    // Area to pass to right side of alliance zone, from perspective of blue alliance station
    public static Region2d passLeftZone =
        new Region2d(
            new Translation2d[] {
              new Translation2d(
                  redLineX - hubOffset, centerLineY + Units.inchesToMeters(47.00 / 2) + bumpLength),
              new Translation2d(
                  blueLineX + hubOffset, centerLineY - Units.inchesToMeters(47.00 / 2)),
              new Translation2d(blueLineX + hubOffset, -fieldEdgeTolerance),
              new Translation2d(redLineX - hubOffset, -fieldEdgeTolerance)
            },
            "blue/passLeftZone");

    // Area to pass to left side of alliance zone, from perspective of blue alliance station
    public static Region2d passRightZone =
        new Region2d(
            new Translation2d[] {
              new Translation2d(
                  redLineX - hubOffset, centerLineY + Units.inchesToMeters(47.00 / 2) + bumpLength),
              new Translation2d(
                  blueLineX + hubOffset, centerLineY - Units.inchesToMeters(47.00 / 2)),
              new Translation2d(blueLineX + hubOffset, fieldWidth + fieldEdgeTolerance),
              new Translation2d(redLineX - hubOffset, fieldWidth + fieldEdgeTolerance)
            },
            "blue/passRightZone");
  }

  public static class Red {
    public static Translation2d hubTranslation =
        new Translation2d(redLineX - hubOffset, centerLineY);
    public static Region2d allianceZone =
        new Region2d(
            new Translation2d(redLineX - hubOffset, -fieldEdgeTolerance),
            new Translation2d(fieldLength + fieldEdgeTolerance, fieldWidth + fieldEdgeTolerance),
            "red/allianceZone");

    public static Region2d rightAllianceZone =
        new Region2d(
            new Translation2d(redLineX - hubOffset, -fieldEdgeTolerance),
            new Translation2d(fieldLength + fieldEdgeTolerance, centerLineY),
            "red/rightAllianceZone");
    public static Region2d leftAllianceZone =
        new Region2d(
            new Translation2d(redLineX - hubOffset, centerLineY),
            new Translation2d(fieldLength + fieldEdgeTolerance, fieldWidth + fieldEdgeTolerance),
            "red/leftAllianceZone");

    // Covers left red trench (only neutral zone side)
    // For use when we are red alliance and want to be able to shoot in the trench
    public static Region2d stopShootLeftNeutral =
        new Region2d(
            new Translation2d(
                redLineX - hubOffset - stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                centerLineY + Units.inchesToMeters(133.47 - 24.97)),
            new Translation2d(
                redLineX - hubOffset + stopShootingZoneBuffer, fieldWidth + fieldEdgeTolerance),
            "red/stopShootLeftNeutral");

    // Covers right red trench (only neutral zone side)
    // For use when we are red alliance and want to be able to shoot in the trench
    public static Region2d stopShootRightNeutral =
        new Region2d(
            new Translation2d(
                redLineX - hubOffset - stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                -fieldEdgeTolerance),
            new Translation2d(
                redLineX - hubOffset + stopShootingZoneBuffer, Units.inchesToMeters(50.59)),
            "red/stopShootRightNeutral");

    // Covers the left red trench (both neutral and alliance zone sides)
    // For use when we are blue alliance and want to stop shooting anywhere near the trench
    public static Region2d stopShootLeftFull =
        new Region2d(
            new Translation2d(
                redLineX - hubOffset - stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                centerLineY + Units.inchesToMeters(133.47 - 24.97)),
            new Translation2d(
                redLineX - hubOffset + stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                fieldWidth + fieldEdgeTolerance),
            "red/stopShootLeftFull");

    // Covers the right red trench (both neutral and alliance zone sides)
    // For use when we are blue alliance and want to stop shooting anywhere near the trench
    public static Region2d stopShootRightFull =
        new Region2d(
            new Translation2d(
                redLineX - hubOffset - stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                -fieldEdgeTolerance),
            new Translation2d(
                redLineX - hubOffset + stopShootingZoneScale * Units.inchesToMeters(22.20 + 6),
                Units.inchesToMeters(50.59)),
            "red/stopShootRightFull");

    public static Region2d frontOfHub =
        new Region2d(
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20),
                centerLineY - Units.inchesToMeters(58.41 / 2)),
            new Translation2d(
                redLineX + Units.inchesToMeters(22.20 + 40),
                centerLineY + Units.inchesToMeters(58.41 / 2)),
            "red/frontOfHub");

    // These zones are based on which side they're passing to, facing the blue driver station
    // Only cover neutral zone

    // Area to pass to left side of alliance zone, from perspective of red alliance station
    public static Region2d passLeftZone =
        new Region2d(
            new Translation2d[] {
              new Translation2d(
                  redLineX - hubOffset, centerLineY + Units.inchesToMeters(47.00 / 2)),
              new Translation2d(
                  blueLineX + hubOffset,
                  centerLineY - Units.inchesToMeters(47.00 / 2) - bumpLength),
              new Translation2d(blueLineX + hubOffset, fieldWidth + fieldEdgeTolerance),
              new Translation2d(redLineX - hubOffset, fieldWidth + fieldEdgeTolerance)
            },
            "red/passRightZone");

    // Area to pass to right side of alliance zone, from perspective of red alliance station
    public static Region2d passRightZone =
        new Region2d(
            new Translation2d[] {
              new Translation2d(
                  redLineX - hubOffset, centerLineY + Units.inchesToMeters(47.00 / 2)),
              new Translation2d(
                  blueLineX + hubOffset,
                  centerLineY - Units.inchesToMeters(47.00 / 2) - bumpLength),
              new Translation2d(blueLineX + hubOffset, -fieldEdgeTolerance),
              new Translation2d(redLineX - hubOffset, -fieldEdgeTolerance)
            },
            "red/passLeftZone");
  }

  public static class Neutral {
    public static Region2d rightNeutral =
        new Region2d(
            new Translation2d(blueLineX + hubOffset, -fieldEdgeTolerance),
            new Translation2d(redLineX - hubOffset, centerLineY),
            "neutral/rightNeutral");
    public static Region2d leftNeutral =
        new Region2d(
            new Translation2d(blueLineX + hubOffset, centerLineY),
            new Translation2d(redLineX - hubOffset, fieldWidth + fieldEdgeTolerance),
            "neutral/leftNeutral");
  }

  public static void plotZones() {
    Blue.allianceZone.logPoints();
    Blue.rightAllianceZone.logPoints();
    Blue.leftAllianceZone.logPoints();
    Blue.stopShootLeftNeutral.logPoints();
    Blue.stopShootRightNeutral.logPoints();
    Blue.stopShootLeftFull.logPoints();
    Blue.stopShootRightFull.logPoints();
    Blue.frontOfHub.logPoints();

    Red.allianceZone.logPoints();
    Red.rightAllianceZone.logPoints();
    Red.leftAllianceZone.logPoints();
    Red.stopShootLeftNeutral.logPoints();
    Red.stopShootRightNeutral.logPoints();
    Red.stopShootLeftFull.logPoints();
    Red.stopShootRightFull.logPoints();
    Red.frontOfHub.logPoints();

    Neutral.rightNeutral.logPoints();
    Neutral.leftNeutral.logPoints();

    Blue.passRightZone.logPoints();
    Blue.passLeftZone.logPoints();

    Red.passRightZone.logPoints();
    Red.passLeftZone.logPoints();
  }

  public static final double aprilTagWidth = Units.inchesToMeters(6.50);
  public static final int aprilTagCount = 32;
  public static final Field aprilTagFieldLayout =
      new Field(
        "aprilTagFieldLayout",
        "2026",
        "Rebuilt",
        null,
        fieldLength,
        fieldWidth,
        "FRC",
          List.of(
              new FieldTag(
                  1,
                  new Pose3d(
                      11.8779798,
                      7.4247756,
                      0.889,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  2,
                  new Pose3d(
                      11.9154194,
                      4.63804,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
              new FieldTag(
                  3,
                  new Pose3d(
                      11.3118646,
                      4.3902376,
                      1.12395,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  4,
                  new Pose3d(
                      11.3118646,
                      4.0346376,
                      1.12395,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  5,
                  new Pose3d(
                      11.9154194,
                      3.4312352,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
              new FieldTag(
                  6,
                  new Pose3d(
                      11.8779798,
                      0.6444996,
                      0.889,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  7,
                  new Pose3d(
                      11.9528844,
                      0.6444996,
                      0.889,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new FieldTag(
                  8,
                  new Pose3d(
                      12.2710194,
                      3.4312352,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
              new FieldTag(
                  9,
                  new Pose3d(
                      12.5191774,
                      3.6790376,
                      1.12395,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new FieldTag(
                  10,
                  new Pose3d(
                      12.5191774,
                      4.0346376,
                      1.12395,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new FieldTag(
                  11,
                  new Pose3d(
                      12.2710194,
                      4.63804,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
              new FieldTag(
                  12,
                  new Pose3d(
                      11.9528844,
                      7.4247756,
                      0.889,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new FieldTag(
                  13,
                  new Pose3d(
                      16.5333172,
                      7.4033126,
                      0.55245,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  14,
                  new Pose3d(
                      16.5333172,
                      6.9715126,
                      0.55245,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  15,
                  new Pose3d(
                      16.5329616,
                      4.3235626,
                      0.55245,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  16,
                  new Pose3d(
                      16.5329616,
                      3.8917626,
                      0.55245,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  17,
                  new Pose3d(
                      4.6630844,
                      0.6444996,
                      0.889,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new FieldTag(
                  18,
                  new Pose3d(
                      4.6256194,
                      3.4312352,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
              new FieldTag(
                  19,
                  new Pose3d(
                      5.2291742,
                      3.6790376,
                      1.12395,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new FieldTag(
                  20,
                  new Pose3d(
                      5.2291742,
                      4.0346376,
                      1.12395,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new FieldTag(
                  21,
                  new Pose3d(
                      4.6256194,
                      4.63804,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
              new FieldTag(
                  22,
                  new Pose3d(
                      4.6630844,
                      7.4247756,
                      0.889,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new FieldTag(
                  23,
                  new Pose3d(
                      4.5881798,
                      7.4247756,
                      0.889,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  24,
                  new Pose3d(
                      4.2700194,
                      4.63804,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(0.7071067811865476, 0.0, 0.0, 0.7071067811865476)))),
              new FieldTag(
                  25,
                  new Pose3d(
                      4.0218614,
                      4.3902376,
                      1.12395,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  26,
                  new Pose3d(
                      4.0218614,
                      4.0346376,
                      1.12395,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  27,
                  new Pose3d(
                      4.2700194,
                      3.4312352,
                      1.12395,
                      new Rotation3d(
                          new Quaternion(-0.7071067811865475, -0.0, 0.0, 0.7071067811865476)))),
              new FieldTag(
                  28,
                  new Pose3d(
                      4.5881798,
                      0.6444996,
                      0.889,
                      new Rotation3d(new Quaternion(6.123233995736766e-17, 0.0, 0.0, 1.0)))),
              new FieldTag(
                  29,
                  new Pose3d(
                      0.007747,
                      0.6659626,
                      0.55245,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new FieldTag(
                  30,
                  new Pose3d(
                      0.007747,
                      1.0977626,
                      0.55245,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new FieldTag(
                  31,
                  new Pose3d(
                      0.0080772,
                      3.7457126,
                      0.55245,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0)))),
              new FieldTag(
                  32,
                  new Pose3d(
                      0.0080772,
                      4.1775126,
                      0.55245,
                      new Rotation3d(new Quaternion(1.0, 0.0, 0.0, 0.0))))));
}
