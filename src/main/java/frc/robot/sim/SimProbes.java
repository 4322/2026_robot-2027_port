package frc.robot.sim;

import frc.robot.RobotContainer;
import frc.robot.sim.scenario.FieldLocations;
import frc.robot.sim.scenario.Pose;
import frc.robot.sim.scenario.ScenarioEvent;
import frc.robot.sim.scenario.SimAlliance;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import java.util.function.Supplier;
import org.wpilib.math.geometry.Pose2d;

/**
 * Named live robot values that scenario {@code expect} events can check.
 *
 * <p>To add a probe, register it in {@link #forRobot()} with an existing getter. The probe name is
 * what scenario files (and the editor's dropdown) use.
 */
final class SimProbes {
  /** Result of evaluating one expect. {@code actual} is a printable observed value. */
  record Outcome(boolean passed, String actual) {}

  private final Map<String, Supplier<Object>> probes = new LinkedHashMap<>();

  void register(String name, Supplier<Object> value) {
    probes.put(name, value);
  }

  Set<String> names() {
    return Collections.unmodifiableSet(probes.keySet());
  }

  /** Every probe the simulator knows about. Keep this list in sync with SimEditor's probe hints. */
  static SimProbes forRobot() {
    SimProbes p = new SimProbes();
    p.register("pose", () -> RobotContainer.drive.getRobotPose());
    p.register("drive.speedMps", () -> RobotContainer.drive.getVelocity().getNorm());
    p.register("intake.state", () -> RobotContainer.intake.getState());
    p.register("intake.extended", () -> RobotContainer.intake.isExtended());
    p.register("intake.smooshed", () -> RobotContainer.intake.isSmooshed());
    p.register("shooter.state", () -> RobotContainer.shooter.getState());
    p.register("shooter.scoring", () -> RobotContainer.shooter.isScoring());
    p.register("shooter.hoodAtGoal", () -> RobotContainer.shooter.hoodAtGoal());
    p.register("shooter.hoodLowered", () -> RobotContainer.shooter.isHoodLowered());
    p.register("shooter.hoodDeg", () -> RobotContainer.shooter.getHoodPositionDegrees());
    p.register("shooter.targetTurretDeg", () -> RobotContainer.shooter.getTargetTurretAngleDeg());
    return p;
  }

  Outcome evaluate(ScenarioEvent.Expect expect, FieldLocations field, SimAlliance alliance) {
    Supplier<Object> supplier = probes.get(expect.probe());
    if (supplier == null) {
      return new Outcome(false, "unknown probe " + expect.probe());
    }
    Object value;
    try {
      value = supplier.get();
    } catch (RuntimeException e) {
      return new Outcome(false, "probe threw " + e);
    }
    if (expect.isPoseCheck()) {
      return evaluatePose(expect, value, field, alliance);
    }
    return evaluateValue(expect, value);
  }

  private static Outcome evaluatePose(
      ScenarioEvent.Expect expect, Object value, FieldLocations field, SimAlliance alliance) {
    if (!(value instanceof Pose2d robot)) {
      return new Outcome(false, "pose probe returned " + value);
    }
    Pose target = field.resolve(expect.location(), expect.pose(), expect.frame(), alliance);
    Pose actual = new Pose(robot.getX(), robot.getY(), robot.getRotation().getDegrees());
    if (target == null) {
      return new Outcome(false, "unknown location " + expect.location());
    }
    double distance = Math.hypot(actual.x() - target.x(), actual.y() - target.y());
    boolean ok = distance <= expect.toleranceOrDefault();
    String detail = String.format(Locale.ROOT, "%s, %.2fm from target", actual, distance);
    if (expect.headingTolDeg() != null) {
      double headingError = Math.abs(Pose.normalizeDeg(actual.deg() - target.deg()));
      ok &= headingError <= expect.headingTolDeg();
      detail += String.format(Locale.ROOT, ", heading off by %.0f°", headingError);
    }
    return new Outcome(ok, detail);
  }

  private static Outcome evaluateValue(ScenarioEvent.Expect expect, Object value) {
    String text = String.valueOf(value);
    boolean ok = true;
    if (expect.equalsValue() != null) {
      ok = text.equalsIgnoreCase(expect.equalsValue());
    }
    if (expect.min() != null || expect.max() != null) {
      if (!(value instanceof Number n)) {
        return new Outcome(false, text + " (not a number)");
      }
      double v = n.doubleValue();
      if (expect.min() != null && v < expect.min()) {
        ok = false;
      }
      if (expect.max() != null && v > expect.max()) {
        ok = false;
      }
      text = String.format(Locale.ROOT, "%.3f", v);
    }
    return new Outcome(ok, text);
  }
}
