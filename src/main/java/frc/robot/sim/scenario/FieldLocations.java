package frc.robot.sim.scenario;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;

/**
 * Contents of {@code simulation/field-locations.json}: field size, named poses that scenarios
 * reference by name, simple field shapes for the editor to draw, and editor preview settings.
 *
 * <p>All coordinates are BLUE-origin field coordinates in metres.
 */
public class FieldLocations {
  /** A rectangle drawn on the editor's field view (hub, trench, bump, ...). Display only. */
  public record Feature(String name, double x1, double y1, double x2, double y2, String color) {}

  private double fieldLength = 16.541;
  private double fieldWidth = 8.069;
  /** Robot speed at full stick, used only to sketch estimated paths in the editor. */
  private double previewMaxSpeedMps = 4.5;
  /** Robot turn rate at full stick, used only for the editor's path estimate. */
  private double previewMaxTurnDegPerSec = 360.0;
  private final Map<String, Pose> locations = new LinkedHashMap<>();
  private final List<Feature> features = new ArrayList<>();

  public double getFieldLength() {
    return fieldLength;
  }

  public void setFieldLength(double fieldLength) {
    this.fieldLength = fieldLength;
  }

  public double getFieldWidth() {
    return fieldWidth;
  }

  public void setFieldWidth(double fieldWidth) {
    this.fieldWidth = fieldWidth;
  }

  public double getPreviewMaxSpeedMps() {
    return previewMaxSpeedMps;
  }

  public void setPreviewMaxSpeedMps(double previewMaxSpeedMps) {
    this.previewMaxSpeedMps = previewMaxSpeedMps;
  }

  public double getPreviewMaxTurnDegPerSec() {
    return previewMaxTurnDegPerSec;
  }

  public void setPreviewMaxTurnDegPerSec(double previewMaxTurnDegPerSec) {
    this.previewMaxTurnDegPerSec = previewMaxTurnDegPerSec;
  }

  /** Live, ordered, mutable map of location name to pose. */
  public Map<String, Pose> locations() {
    return locations;
  }

  /** Live, mutable list of display shapes. */
  public List<Feature> features() {
    return features;
  }

  /**
   * Resolves a pose for the given alliance.
   *
   * @param location named location, or null to use {@code pose}
   * @param pose explicit pose (used when location is null)
   * @param frame ALLIANCE flips for RED; ABSOLUTE never flips
   * @return the field pose, or null if the location name is unknown
   */
  public Pose resolve(String location, Pose pose, ScenarioEvent.Frame frame, SimAlliance alliance) {
    Pose base = location != null ? locations.get(location) : pose;
    if (base == null) {
      return null;
    }
    boolean flip = alliance == SimAlliance.RED && frame != ScenarioEvent.Frame.ABSOLUTE;
    return flip ? base.flipped(fieldLength, fieldWidth) : base;
  }
}
