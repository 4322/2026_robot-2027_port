package frc.robot.sim.scenario;

import java.util.List;

/**
 * The probe names {@code expect} events may use, with a short description for the editor. The
 * simulator's {@code SimProbes} registers a value supplier for each and warns at startup if this
 * list and the registrations drift apart.
 */
public final class ProbeCatalog {
  private ProbeCatalog() {}

  public enum Kind {
    /** Compared with location/x,y + tolerance. */
    POSE,
    /** Compared with min / max. */
    NUMBER,
    /** Compared with equals true/false. */
    BOOLEAN,
    /** Compared with equals NAME. */
    ENUM
  }

  public record Probe(String name, Kind kind, String description, List<String> values) {}

  public static final List<Probe> PROBES = List.of(
      new Probe("pose", Kind.POSE, "Robot pose (odometry) is near a location", List.of()),
      new Probe("drive.speedMps", Kind.NUMBER, "Robot translational speed, m/s", List.of()),
      new Probe("intake.state", Kind.ENUM, "Intake state machine",
          List.of("STARING_CONFIG", "DEPLOY", "IDLE", "EJECT", "SMOOSH", "INTAKING")),
      new Probe("intake.extended", Kind.BOOLEAN, "Intake deployer is extended", List.of()),
      new Probe("intake.smooshed", Kind.BOOLEAN, "Intake is in the smooshed position", List.of()),
      new Probe("shooter.state", Kind.ENUM, "Shooter state machine",
          List.of("STARTING_CONFIG", "DISABLED", "IDLE", "UNWIND", "PRESHOOT", "SHOOT", "STOP")),
      new Probe("shooter.scoring", Kind.BOOLEAN, "Shooter is aiming at the hub (vs passing)", List.of()),
      new Probe("shooter.hoodAtGoal", Kind.BOOLEAN, "Hood has reached its target angle", List.of()),
      new Probe("shooter.hoodLowered", Kind.BOOLEAN, "Hood is lowered (e.g. for the trench)", List.of()),
      new Probe("shooter.hoodDeg", Kind.NUMBER, "Hood angle, degrees", List.of()),
      new Probe("shooter.targetTurretDeg", Kind.NUMBER, "Turret target angle, degrees", List.of()));

  public static List<String> names() {
    return PROBES.stream().map(Probe::name).toList();
  }

  public static Probe find(String name) {
    return PROBES.stream().filter(p -> p.name().equals(name)).findFirst().orElse(null);
  }
}
