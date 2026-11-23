package frc.robot.sim.scenario;

import java.util.Locale;

/**
 * One step of a scenario. Every event runs {@link #waitSec()} seconds after the previous event in
 * the same segment (auto or teleop), matching the old {@code t += ...} authoring style.
 *
 * <p>Events are immutable records; editors replace an event rather than mutating it.
 */
public sealed interface ScenarioEvent {

  /** Seconds after the previous event (or segment start) at which this event runs. */
  double waitSec();

  /** Optional human-readable name, logged to {@code Sim/EventName}. May be null. */
  String label();

  /** The JSON {@code "type"} value for this event. */
  String type();

  /** Returns a copy of this event with a different wait. */
  ScenarioEvent withWait(double waitSec);

  /** Returns a copy of this event with a different label. */
  ScenarioEvent withLabel(String label);

  /** One-line description of the event's parameters, for tables and logs. */
  String summary();

  /** Label if present, otherwise a generated description. */
  default String displayName() {
    return label() != null && !label().isBlank() ? label() : type() + " " + summary();
  }

  // ---------------------------------------------------------------- enums

  /** Xbox buttons, in {@code NiDsXboxController.Button} order (index 0..9). */
  enum GamepadButton {
    A,
    B,
    X,
    Y,
    LEFT_BUMPER,
    RIGHT_BUMPER,
    BACK,
    START,
    LEFT_STICK,
    RIGHT_STICK
  }

  enum ButtonAction {
    /** Down for exactly one robot loop, then released automatically. */
    PRESS,
    /** Down until a matching RELEASE. */
    HOLD,
    RELEASE
  }

  /** POV directions, by {@code POVDirection} name. */
  enum PovDirection {
    CENTER,
    UP,
    UP_RIGHT,
    RIGHT,
    DOWN_RIGHT,
    DOWN,
    DOWN_LEFT,
    LEFT,
    UP_LEFT
  }

  enum TriggerSide {
    LEFT,
    RIGHT
  }

  /** How a pose in a scenario relates to the alliance being simulated. */
  enum Frame {
    /** Written for BLUE; point-flipped when the test runs as RED. */
    ALLIANCE,
    /** Exact field coordinates, never flipped. */
    ABSOLUTE
  }

  // ---------------------------------------------------------------- events

  /** Teleport the robot. Uses a named location from field-locations.json, or an explicit pose. */
  record SetPose(double waitSec, String label, String location, Pose pose, Frame frame)
      implements ScenarioEvent {
    public String type() {
      return "setPose";
    }

    public SetPose withWait(double w) {
      return new SetPose(w, label, location, pose, frame);
    }

    public SetPose withLabel(String l) {
      return new SetPose(waitSec, l, location, pose, frame);
    }

    public String summary() {
      String where = location != null ? location : String.valueOf(pose);
      return frame == Frame.ABSOLUTE ? where + " [absolute]" : where;
    }
  }

  record Button(double waitSec, String label, GamepadButton button, ButtonAction action)
      implements ScenarioEvent {
    public String type() {
      return "button";
    }

    public Button withWait(double w) {
      return new Button(w, label, button, action);
    }

    public Button withLabel(String l) {
      return new Button(waitSec, l, button, action);
    }

    public String summary() {
      return lower(action) + " " + button;
    }
  }

  /** POV hat. RELEASE / CENTER both return the hat to centre. */
  record Pov(double waitSec, String label, PovDirection direction, ButtonAction action)
      implements ScenarioEvent {
    public String type() {
      return "pov";
    }

    public Pov withWait(double w) {
      return new Pov(w, label, direction, action);
    }

    public Pov withLabel(String l) {
      return new Pov(waitSec, l, direction, action);
    }

    public String summary() {
      return lower(action) + " " + direction;
    }
  }

  /** Analog trigger, 0 (released) to 1 (fully pulled). Persists until changed. */
  record Trigger(double waitSec, String label, TriggerSide side, double value)
      implements ScenarioEvent {
    public String type() {
      return "trigger";
    }

    public Trigger withWait(double w) {
      return new Trigger(w, label, side, value);
    }

    public Trigger withLabel(String l) {
      return new Trigger(waitSec, l, side, value);
    }

    public String summary() {
      return side + " = " + fmt(value);
    }
  }

  /**
   * Left stick translation, driver-relative: x = forward, y = left, each -1..1. The robot code
   * already flips for RED, so the same values mean "away from my driver station" on both alliances.
   * Persists until changed.
   */
  record Drive(double waitSec, String label, double x, double y) implements ScenarioEvent {
    public String type() {
      return "drive";
    }

    public Drive withWait(double w) {
      return new Drive(w, label, x, y);
    }

    public Drive withLabel(String l) {
      return new Drive(waitSec, l, x, y);
    }

    public String summary() {
      return "fwd " + fmt(x) + ", left " + fmt(y);
    }
  }

  /** Right stick rotation, -1..1, positive = counter-clockwise. Persists until changed. */
  record Turn(double waitSec, String label, double rate) implements ScenarioEvent {
    public String type() {
      return "turn";
    }

    public Turn withWait(double w) {
      return new Turn(w, label, rate);
    }

    public Turn withLabel(String l) {
      return new Turn(waitSec, l, rate);
    }

    public String summary() {
      return "ccw " + fmt(rate);
    }
  }

  /** Centre both sticks. */
  record StopJoysticks(double waitSec, String label) implements ScenarioEvent {
    public String type() {
      return "stopJoysticks";
    }

    public StopJoysticks withWait(double w) {
      return new StopJoysticks(w, label);
    }

    public StopJoysticks withLabel(String l) {
      return new StopJoysticks(waitSec, l);
    }

    public String summary() {
      return "";
    }
  }

  /** FMS game-specific message ("B" = blue inactive first, "R" = red inactive first). */
  record GameData(double waitSec, String label, String value) implements ScenarioEvent {
    public String type() {
      return "gameData";
    }

    public GameData withWait(double w) {
      return new GameData(w, label, value);
    }

    public GameData withLabel(String l) {
      return new GameData(waitSec, l, value);
    }

    public String summary() {
      return "\"" + value + "\"";
    }
  }

  /**
   * An assertion on a named probe (see {@code SimProbes}).
   *
   * <ul>
   *   <li>{@code probe = "pose"}: robot within {@code tolerance} metres of {@code location} or
   *       {@code pose} (and within {@code headingTolDeg} if given).
   *   <li>otherwise: value {@code equals} a string (enum name / true / false), and/or lies in
   *       [{@code min}, {@code max}].
   * </ul>
   *
   * If {@code within} is set the check passes as soon as it is true at any point in the next
   * {@code within} seconds; otherwise it is checked once, immediately.
   */
  record Expect(
      double waitSec,
      String label,
      String probe,
      String equalsValue,
      Double min,
      Double max,
      String location,
      Pose pose,
      Frame frame,
      Double tolerance,
      Double headingTolDeg,
      Double within)
      implements ScenarioEvent {

    public static final double DEFAULT_POSE_TOLERANCE = 0.25;

    public String type() {
      return "expect";
    }

    public Expect withWait(double w) {
      return new Expect(
          w, label, probe, equalsValue, min, max, location, pose, frame, tolerance, headingTolDeg,
          within);
    }

    public Expect withLabel(String l) {
      return new Expect(
          waitSec, l, probe, equalsValue, min, max, location, pose, frame, tolerance,
          headingTolDeg, within);
    }

    public boolean isPoseCheck() {
      return "pose".equals(probe);
    }

    public double toleranceOrDefault() {
      return tolerance != null ? tolerance : DEFAULT_POSE_TOLERANCE;
    }

    public String summary() {
      StringBuilder sb = new StringBuilder(String.valueOf(probe));
      if (isPoseCheck()) {
        sb.append(" near ")
            .append(location != null ? location : String.valueOf(pose))
            .append(" ±")
            .append(fmt(toleranceOrDefault()))
            .append("m");
        if (headingTolDeg != null) {
          sb.append(" ±").append(fmt(headingTolDeg)).append("°");
        }
      } else {
        if (equalsValue != null) {
          sb.append(" == ").append(equalsValue);
        }
        if (min != null) {
          sb.append(" >= ").append(fmt(min));
        }
        if (max != null) {
          sb.append(" <= ").append(fmt(max));
        }
      }
      if (within != null) {
        sb.append(" within ").append(fmt(within)).append("s");
      }
      return sb.toString();
    }
  }

  /** Marks the end of a segment. Must be the last event if present. */
  record End(double waitSec, String label) implements ScenarioEvent {
    public String type() {
      return "end";
    }

    public End withWait(double w) {
      return new End(w, label);
    }

    public End withLabel(String l) {
      return new End(waitSec, l);
    }

    public String summary() {
      return "";
    }
  }

  // ---------------------------------------------------------------- helpers

  private static String lower(Enum<?> e) {
    return e.name().toLowerCase(Locale.ROOT);
  }

  /** Compact number formatting: 0.5, 1, -0.25. */
  static String fmt(double v) {
    if (v == Math.rint(v)) {
      return String.valueOf((long) v);
    }
    return String.format(Locale.ROOT, "%.3f", v).replaceAll("0+$", "");
  }
}
