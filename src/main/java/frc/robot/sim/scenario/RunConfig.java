package frc.robot.sim.scenario;

import java.util.ArrayList;
import java.util.List;

/**
 * Contents of {@code simulation/run.json}: what the next simulator launch should do. Written by
 * the editor GUI. If the file is missing the simulator runs the {@code default} suite.
 */
public class RunConfig {
  public enum Mode {
    /** Run scripted scenarios (a suite, or a list of scenarios × alliances). */
    SCRIPTED,
    /** Set alliance / mode / start pose, then leave the controller and enable to the user. */
    MANUAL
  }

  public enum ManualRobotMode {
    AUTONOMOUS,
    TELEOPERATED
  }

  public static final double DEFAULT_WARMUP_SEC = 4.0;
  public static final double DEFAULT_GAP_SEC = 2.0;
  public static final String DEFAULT_SUITE = "default";

  private Mode mode = Mode.SCRIPTED;

  // SCRIPTED: either a suite id, or a list of scenario ids run on each of `alliances`.
  private String suite;
  private final List<String> scenarios = new ArrayList<>();
  private final List<SimAlliance> alliances = new ArrayList<>();

  // MANUAL
  private SimAlliance manualAlliance = SimAlliance.BLUE;
  private ManualRobotMode manualRobotMode = ManualRobotMode.TELEOPERATED;
  private String startLocation;
  private String autoRoutine;

  /** Seconds to wait after robot code starts before the first test (libraries initializing). */
  private double warmupSec = DEFAULT_WARMUP_SEC;
  /** Seconds the robot stays disabled between segments/tests. */
  private double gapSec = DEFAULT_GAP_SEC;

  public static RunConfig defaultSuite() {
    RunConfig c = new RunConfig();
    c.suite = DEFAULT_SUITE;
    return c;
  }

  public Mode getMode() {
    return mode;
  }

  public void setMode(Mode mode) {
    this.mode = mode;
  }

  public String getSuite() {
    return suite;
  }

  public void setSuite(String suite) {
    this.suite = suite;
  }

  public List<String> scenarios() {
    return scenarios;
  }

  public List<SimAlliance> alliances() {
    return alliances;
  }

  public SimAlliance getManualAlliance() {
    return manualAlliance;
  }

  public void setManualAlliance(SimAlliance manualAlliance) {
    this.manualAlliance = manualAlliance;
  }

  public ManualRobotMode getManualRobotMode() {
    return manualRobotMode;
  }

  public void setManualRobotMode(ManualRobotMode manualRobotMode) {
    this.manualRobotMode = manualRobotMode;
  }

  public String getStartLocation() {
    return startLocation;
  }

  public void setStartLocation(String startLocation) {
    this.startLocation = startLocation;
  }

  public String getAutoRoutine() {
    return autoRoutine;
  }

  public void setAutoRoutine(String autoRoutine) {
    this.autoRoutine = autoRoutine;
  }

  public double getWarmupSec() {
    return warmupSec;
  }

  public void setWarmupSec(double warmupSec) {
    this.warmupSec = warmupSec;
  }

  public double getGapSec() {
    return gapSec;
  }

  public void setGapSec(double gapSec) {
    this.gapSec = gapSec;
  }
}
