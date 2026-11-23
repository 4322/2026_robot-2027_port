package frc.robot.sim.scenario;

/**
 * A field pose in metres / degrees, in BLUE-origin field coordinates.
 *
 * <p>Kept free of WPILib types so the scenario editor GUI can use it without loading native
 * libraries. The simulator converts it to a {@code Pose2d}.
 */
public record Pose(double x, double y, double deg) {

  /**
   * Returns this pose as seen from the other alliance. The 2026 field is rotationally symmetric,
   * so this is a point reflection through the field centre plus a 180 degree turn.
   */
  public Pose flipped(double fieldLength, double fieldWidth) {
    return new Pose(fieldLength - x, fieldWidth - y, normalizeDeg(deg + 180.0));
  }

  public Pose withX(double newX) {
    return new Pose(newX, y, deg);
  }

  public Pose withY(double newY) {
    return new Pose(x, newY, deg);
  }

  public Pose withDeg(double newDeg) {
    return new Pose(x, y, normalizeDeg(newDeg));
  }

  /** Normalizes an angle to the range (-180, 180]. */
  public static double normalizeDeg(double deg) {
    double d = deg % 360.0;
    if (d <= -180.0) {
      d += 360.0;
    } else if (d > 180.0) {
      d -= 360.0;
    }
    return d;
  }

  @Override
  public String toString() {
    return String.format("(%.2f, %.2f, %.0f°)", x, y, deg);
  }
}
