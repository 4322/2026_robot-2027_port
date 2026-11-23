package frc.robot.sim.scenario;

/**
 * A scripted simulation run: an optional auto segment followed by an optional teleop segment.
 * Stored as one JSON file in {@code simulation/scenarios/}.
 *
 * <p>Mutable so the editor GUI can change it in place.
 */
public class Scenario {
  private String name;
  private String description = "";
  /** Game-specific message at the start of the scenario ("", "B" or "R"). */
  private String gameData = "";
  /** Log every controller button edge to {@code Sim/Debug} (useful for controller tests). */
  private boolean debugBindings;
  private Segment auto;
  private Segment teleop;

  public Scenario(String name) {
    this.name = name;
  }

  /** Builds an auto-only scenario for one routine (used by suites with {@code "autos": "ALL"}). */
  public static Scenario autoOnly(String routine, Double durationSec) {
    Scenario s = new Scenario(routine);
    s.auto = new Segment(true);
    s.auto.setAutoRoutine(routine);
    s.auto.setDurationSec(durationSec);
    return s;
  }

  public String getName() {
    return name;
  }

  public void setName(String name) {
    this.name = name;
  }

  public String getDescription() {
    return description;
  }

  public void setDescription(String description) {
    this.description = description == null ? "" : description;
  }

  public String getGameData() {
    return gameData;
  }

  public void setGameData(String gameData) {
    this.gameData = gameData == null ? "" : gameData;
  }

  public boolean isDebugBindings() {
    return debugBindings;
  }

  public void setDebugBindings(boolean debugBindings) {
    this.debugBindings = debugBindings;
  }

  /** Auto segment, or null if the scenario has no auto. */
  public Segment getAuto() {
    return auto;
  }

  public void setAuto(Segment auto) {
    this.auto = auto;
  }

  /** Teleop segment, or null if the scenario has no teleop. */
  public Segment getTeleop() {
    return teleop;
  }

  public void setTeleop(Segment teleop) {
    this.teleop = teleop;
  }

  public Scenario copy() {
    Scenario s = new Scenario(name);
    s.description = description;
    s.gameData = gameData;
    s.debugBindings = debugBindings;
    s.auto = auto == null ? null : auto.copy();
    s.teleop = teleop == null ? null : teleop.copy();
    return s;
  }
}
