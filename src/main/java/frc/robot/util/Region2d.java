package frc.robot.util;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import java.awt.geom.*;
import org.littletonrobotics.junction.Logger;

/**
 * This class models a region of the field. It is defined by its vertices and the transition points
 * to neighboring regions. Created by FRC 3061
 * https://github.com/HuskieRobotics/3061-lib/blob/main/src/main/java/frc/robot/Region2d.java
 */
public class Region2d {
  private Path2D shape;
  private String id;
  private Pose2d[] points;

  /**
   * Create a Region2d, a polygon, from an array of Translation2d specifying vertices of a polygon.
   * The polygon is created using the even-odd winding rule.
   *
   * @param points the array of Translation2d that define the vertices of the region.
   * @param id the identifier for the region. (for logging purposes)
   */
  public Region2d(Translation2d[] points, String id) {
    this.points = new Pose2d[points.length + 1];
    for (int i = 0; i < points.length; i++) {
      this.points[i] = new Pose2d(points[i], Rotation2d.ZERO);
    }
    this.points[points.length] = new Pose2d(points[0], Rotation2d.ZERO); // Close the path
    this.id = id;
    this.shape = new Path2D.Double(Path2D.WIND_EVEN_ODD, points.length);
    this.shape.moveTo(points[0].getX(), points[0].getY());

    for (int i = 1; i < points.length; i++) {
      this.shape.lineTo(points[i].getX(), points[i].getY());
    }

    this.shape.closePath();
  }

  /**
   * Create a rectangular Region2d from two opposite corners.
   *
   * @param cornerA one rectangle corner
   * @param cornerB opposite rectangle corner
   * @param id the identifier for the region. (for logging purposes)
   */
  public Region2d(Translation2d cornerA, Translation2d cornerB, String id) {
    this(
        new Translation2d[] {
          new Translation2d(
              Math.min(cornerA.getX(), cornerB.getX()), Math.min(cornerA.getY(), cornerB.getY())),
          new Translation2d(
              Math.max(cornerA.getX(), cornerB.getX()), Math.min(cornerA.getY(), cornerB.getY())),
          new Translation2d(
              Math.max(cornerA.getX(), cornerB.getX()), Math.max(cornerA.getY(), cornerB.getY())),
          new Translation2d(
              Math.min(cornerA.getX(), cornerB.getX()), Math.max(cornerA.getY(), cornerB.getY()))
        },
        id);
  }

  /**
   * Log the bounding rectangle of the region and the transition points to neighboring regions.
   * These can be visualized using AdvantageScope to confirm that the regions are properly defined.
   */
  public void logPoints() {
    Logger.recordOutput("Region2d/" + id, points);
  }

  /**
   * Returns true if the region contains a given Pose2d.
   *
   * @param other the given pose2d
   * @return if the pose is inside the region
   */
  public boolean contains(Pose2d other) {

    return this.shape.contains(new Point2D.Double(other.getX(), other.getY()));
  }
}
