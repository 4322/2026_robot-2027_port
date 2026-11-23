package frc.robot.sim;

import frc.robot.RobotContainer;
import frc.robot.autonomous.AutonomousSelector.AutoName;
import frc.robot.sim.scenario.FieldLocations;
import frc.robot.sim.scenario.Pose;
import frc.robot.sim.scenario.ProbeCatalog;
import frc.robot.sim.scenario.RunConfig;
import frc.robot.sim.scenario.Scenario;
import frc.robot.sim.scenario.ScenarioEvent;
import frc.robot.sim.scenario.ScenarioJson;
import frc.robot.sim.scenario.Segment;
import frc.robot.sim.scenario.SimAlliance;
import frc.robot.sim.scenario.SimPaths;
import frc.robot.sim.scenario.Suite;
import java.io.IOException;
import java.nio.file.Files;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;
import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.CommandScheduler;
import org.wpilib.command2.Commands;
import org.wpilib.command2.button.Trigger;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.hardware.hal.RobotMode;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.system.Timer;

/**
 * Scripted regression runner for simulation. Drives a virtual Xbox controller and the simulated
 * driver station through JSON scenarios in {@code simulation/}, checks {@code expect} events, and
 * writes {@code simulation/results/latest.json}. Robot output is viewed in AdvantageScope.
 *
 * <p>What to run comes from {@code simulation/run.json} (written by the editor GUI, {@code
 * ./gradlew simEditor}); without it the {@code default} suite runs. In MANUAL mode the simulator
 * only sets alliance / mode / start pose and leaves the controller to a real gamepad.
 *
 * <p>Lifecycle per test: WARMUP (first test only) → for each segment: GAP (disabled) → RUNNING.
 * After the last test the report is written and the JVM exits 0 (all passed) or 1.
 *
 * <p>Called from {@code Robot.robotPeriodic()} before the command scheduler runs.
 */
public class Simulator {
  private enum Phase {
    /** Robot code just started; give libraries time to initialize. */
    WARMUP,
    /** Robot disabled between segments / tests. */
    GAP,
    /** A segment is enabled and its events are running. */
    RUNNING,
    /** Manual gamepad mode: nothing scripted. */
    MANUAL,
    DONE
  }

  /** One scenario × alliance. {@code scenarioId} is null for generated all-autos tests. */
  private record TestCase(String name, String scenarioId, Scenario scenario, SimAlliance alliance) {}

  /** An expect waiting up to {@code deadline} to become true. */
  private record PendingExpect(ScenarioEvent.Expect expect, int index, double deadline, String label) {}

  private static Simulator active;
  private static AutoName requestedAuto;

  private final SimPaths paths = SimPaths.locate();
  private final SimResults results = new SimResults();
  private final SimProbes probes = SimProbes.forRobot();
  private final VirtualGamepad gamepad = new VirtualGamepad(RobotContainer.controller.getPort());
  private final EventExecutor executor = new EventExecutor(gamepad, this::setPose);
  private final Timer phaseTimer = new Timer();
  private final List<PendingExpect> pending = new ArrayList<>();

  private RunConfig config;
  private FieldLocations field = new FieldLocations();
  private Iterator<TestCase> tests = List.<TestCase>of().iterator();
  private Phase phase = Phase.WARMUP;
  private TestCase test;
  private Segment segment;
  private double[] eventTimes;
  private int nextEvent;
  private boolean debugBindingsRegistered;

  public Simulator() {
    active = this;
    checkProbeCatalog();
    loadRun();
    phaseTimer.start();
  }

  // ================================================================== static API for robot code

  /** The auto routine the simulator wants run, if any. Read by {@code AutonomousSelector}. */
  public static Optional<AutoName> requestedAuto() {
    return Optional.ofNullable(requestedAuto);
  }

  /** Lets robot code report a simulation failure (e.g. requested auto not found). */
  public static void reportError(String message) {
    if (active != null) {
      active.results.testError(message);
    } else {
      System.err.println("[Sim] ERROR: " + message);
    }
  }

  // ================================================================== loading

  private void loadRun() {
    try {
      field = ScenarioJson.readFieldLocations(paths.fieldLocations());
    } catch (IOException e) {
      results.runError("Could not read field locations: " + e.getMessage());
    }
    try {
      config = Files.exists(paths.runConfig())
          ? ScenarioJson.readRunConfig(paths.runConfig())
          : RunConfig.defaultSuite();
    } catch (IOException e) {
      results.runError("Could not read run config: " + e.getMessage());
      config = new RunConfig();
      config.setSuite(RunConfig.DEFAULT_SUITE);
      return;
    }
    System.out.println("[Sim] simulation dir: " + paths.dir() + ", mode " + config.getMode()
        + (config.getSuite() != null ? ", suite " + config.getSuite() : ""));
    if (config.getMode() == RunConfig.Mode.SCRIPTED) {
      tests = buildTests().iterator();
    }
  }

  private List<TestCase> buildTests() {
    List<TestCase> out = new ArrayList<>();
    Map<String, Scenario> cache = new HashMap<>();
    if (config.getSuite() != null) {
      Suite suite;
      try {
        suite = ScenarioJson.readSuite(paths.suite(config.getSuite()));
      } catch (IOException e) {
        results.runError("Could not read suite \"" + config.getSuite() + "\": " + e.getMessage());
        return out;
      }
      for (Suite.Run run : suite.runs()) {
        if (run.allAutos()) {
          for (SimAlliance alliance : run.alliances()) {
            for (AutoName auto : AutoName.values()) {
              if (!auto.name().startsWith("DRIVE_")) { // skip characterization routines
                out.add(new TestCase("Auto " + auto, null, Scenario.autoOnly(auto.name(), run.autoDuration()), alliance));
              }
            }
          }
        } else {
          addScenario(out, cache, run.scenario(), run.alliances());
        }
      }
    } else {
      for (String id : config.scenarios()) {
        addScenario(out, cache, id, config.alliances());
      }
    }
    return out;
  }

  private void addScenario(List<TestCase> out, Map<String, Scenario> cache, String id, List<SimAlliance> alliances) {
    Scenario s = cache.computeIfAbsent(id, this::loadScenario);
    if (s == null) {
      return;
    }
    for (SimAlliance alliance : alliances) {
      out.add(new TestCase(s.getName(), id, s, alliance));
    }
  }

  /** Reads and validates one scenario; returns null (and records why) if it can't be run. */
  private Scenario loadScenario(String id) {
    try {
      Scenario s = ScenarioJson.readScenario(paths.scenario(id));
      Set<String> autos = Arrays.stream(AutoName.values()).map(Enum::name).collect(Collectors.toSet());
      List<String> problems = ScenarioJson.validate(s, field, autos, probes.names());
      if (!problems.isEmpty()) {
        results.runError("Scenario \"" + id + "\" is invalid:\n    " + String.join("\n    ", problems));
        return null;
      }
      return s;
    } catch (IOException e) {
      results.runError("Could not read scenario \"" + id + "\": " + e.getMessage());
      return null;
    }
  }

  private void checkProbeCatalog() {
    if (!Set.copyOf(ProbeCatalog.names()).equals(probes.names())) {
      System.err.println("[Sim] Warning: ProbeCatalog and SimProbes list different probes: "
          + ProbeCatalog.names() + " vs " + probes.names());
    }
  }

  // ================================================================== main loop

  public void periodic() {
    if (phase != Phase.DONE && Files.exists(paths.stopFlag())) {
      try {
        Files.deleteIfExists(paths.stopFlag());
      } catch (IOException ignored) {
        // the GUI will clean it up
      }
      System.out.println("[Sim] Stop requested");
      finish(true);
      return;
    }
    switch (phase) {
      case WARMUP -> {
        if (config.getMode() == RunConfig.Mode.MANUAL) {
          if (phaseTimer.hasElapsed(config.getWarmupSec())) {
            startManual();
          }
          return; // never touch the controller in manual mode
        }
        if (phaseTimer.hasElapsed(config.getWarmupSec())) {
          nextTest();
        }
      }
      case GAP -> {
        if (phaseTimer.hasElapsed(config.getGapSec())) {
          startSegment();
        }
      }
      case RUNNING -> runSegment();
      case MANUAL -> {
        manualPeriodic();
        return; // never touch the controller in manual mode
      }
      case DONE -> {
        return;
      }
    }
    if (phase != Phase.DONE && !gamepad.flush()) {
      results.testError(String.format("controller update failed at t=%.2f; skipping to next test", phaseTimer.get()));
      nextTest();
    }
  }

  // ================================================================== test / segment lifecycle

  private void nextTest() {
    gamepad.reset();
    pending.clear();
    CommandScheduler.getInstance().cancelAll();
    DriverStationSim.setEnabled(false);
    if (!tests.hasNext()) {
      finish(false);
      return;
    }
    test = tests.next();
    results.beginTest(test.name(), test.scenarioId(), test.alliance());
    gamepad.gameData(test.scenario().getGameData());
    if (test.scenario().isDebugBindings()) {
      registerDebugBindings();
    }
    setRobotPose(new Pose(0, 0, 0));
    System.out.println("[Sim] ---- " + test.name() + " (" + test.alliance() + ")");
    prepareSegment(test.scenario().getAuto() != null ? test.scenario().getAuto() : test.scenario().getTeleop());
  }

  /** Disables the robot and configures the DS for the segment; it starts after the gap. */
  private void prepareSegment(Segment next) {
    segment = next;
    eventTimes = next.absoluteTimes();
    nextEvent = 0;
    pending.clear();
    requestedAuto = next.isAuto() ? AutoName.valueOf(next.getAutoRoutine()) : null;
    DriverStationSim.setEnabled(false);
    DriverStationSim.setRobotMode(next.isAuto() ? RobotMode.AUTONOMOUS : RobotMode.TELEOPERATED);
    DriverStationSim.setAllianceStationId(
        test.alliance() == SimAlliance.RED ? AllianceStationID.RED_2 : AllianceStationID.BLUE_2);
    DriverStationSim.setMatchTime(next.effectiveStartMatchTime());
    Logger.recordOutput("Sim/RegressionTest", "Disabled");
    Logger.recordOutput("Sim/Scenario", "Disabled");
    Logger.recordOutput("Sim/EventName", "Disabled");
    Logger.recordOutput("Sim/EventType", "Disabled");
    phase = Phase.GAP;
    phaseTimer.restart();
  }

  private void startSegment() {
    DriverStationSim.setEnabled(true);
    Logger.recordOutput("Sim/RegressionTest", test.name());
    Logger.recordOutput("Sim/Alliance", test.alliance().toString());
    Logger.recordOutput("Sim/Scenario", test.name() + (segment.isAuto() ? " (auto)" : " (teleop)"));
    phase = Phase.RUNNING;
    phaseTimer.restart();
  }

  private void runSegment() {
    double t = phaseTimer.get();
    Logger.recordOutput("Sim/MatchTime", t);
    DriverStationSim.setMatchTime(Math.max(0.0, segment.effectiveStartMatchTime() - t));

    List<ScenarioEvent> events = segment.events();
    while (nextEvent < events.size() && t >= eventTimes[nextEvent]) {
      ScenarioEvent e = events.get(nextEvent);
      Logger.recordOutput("Sim/EventName", e.displayName());
      Logger.recordOutput("Sim/EventType", e.type());
      if (e instanceof ScenarioEvent.Expect x) {
        startExpect(x, nextEvent, t);
      } else {
        executor.execute(e);
      }
      nextEvent++;
    }
    checkPendingExpects(t, false);

    if (nextEvent >= events.size() && t >= segment.endTime()) {
      checkPendingExpects(t, true);
      if (segment.isAuto() && test.scenario().getTeleop() != null) {
        prepareSegment(test.scenario().getTeleop());
      } else {
        nextTest();
      }
    }
  }

  // ================================================================== expects

  private String segmentName() {
    return segment.isAuto() ? "auto" : "teleop";
  }

  private void startExpect(ScenarioEvent.Expect x, int index, double t) {
    String label = x.label() != null ? x.label() : "expect " + x.summary();
    SimProbes.Outcome outcome = probes.evaluate(x, field, test.alliance());
    if (outcome.passed() || x.within() == null) {
      results.check(segmentName(), index, label, outcome.passed(), outcome.actual(), t);
    } else {
      pending.add(new PendingExpect(x, index, t + x.within(), label));
    }
  }

  /** Resolves waiting expects that became true, or expired (or all of them, at segment end). */
  private void checkPendingExpects(double t, boolean segmentEnded) {
    for (Iterator<PendingExpect> it = pending.iterator(); it.hasNext(); ) {
      PendingExpect p = it.next();
      SimProbes.Outcome outcome = probes.evaluate(p.expect(), field, test.alliance());
      if (outcome.passed() || t >= p.deadline() || segmentEnded) {
        String actual = outcome.passed() || t >= p.deadline() ? outcome.actual() : outcome.actual() + " (segment ended)";
        results.check(segmentName(), p.index(), p.label(), outcome.passed(), actual, t);
        it.remove();
      }
    }
  }

  // ================================================================== manual mode

  private void startManual() {
    requestedAuto = null;
    if (config.getAutoRoutine() != null) {
      try {
        requestedAuto = AutoName.valueOf(config.getAutoRoutine());
      } catch (IllegalArgumentException e) {
        results.runError("Unknown auto routine \"" + config.getAutoRoutine() + "\"; using the dashboard selection");
      }
    }
    DriverStationSim.setEnabled(false);
    DriverStationSim.setRobotMode(
        config.getManualRobotMode() == RunConfig.ManualRobotMode.AUTONOMOUS ? RobotMode.AUTONOMOUS : RobotMode.TELEOPERATED);
    DriverStationSim.setAllianceStationId(
        config.getManualAlliance() == SimAlliance.RED ? AllianceStationID.RED_2 : AllianceStationID.BLUE_2);
    if (config.getStartLocation() != null) {
      Pose start = field.resolve(config.getStartLocation(), null, ScenarioEvent.Frame.ALLIANCE, config.getManualAlliance());
      if (start == null) {
        results.runError("Unknown start location \"" + config.getStartLocation() + "\"");
      } else {
        setRobotPose(start);
      }
    }
    DriverStationSim.notifyNewData();
    gamepad.leaveGameDataAlone();
    Logger.recordOutput("Sim/RegressionTest", "Manual");
    Logger.recordOutput("Sim/Alliance", config.getManualAlliance().toString());
    System.out.println("[Sim] Manual mode: in the sim GUI, drag your gamepad from System Joysticks onto Joystick "
        + MANUAL_SOURCE_PORT + " (not 0), tick \"Map gamepad\", then in Robot State select Teleoperated AND click Enable.");
    phase = Phase.MANUAL;
  }

  /**
   * Sim GUI joystick slot for the user's controller in manual mode.
   *
   * <p>The 2027 sim GUI's "Map gamepad" produces the 2027 Gamepad layout, but the robot code reads
   * the old NI DS Xbox layout ({@code NiDsXboxController}), where only the left stick and A/B/X/Y
   * line up. So the controller goes on this slot and {@link #relayManualController()} converts it
   * onto the robot's port every loop (see {@link GamepadLayout}).
   */
  static final int MANUAL_SOURCE_PORT = 1;

  private boolean relaying;

  /** Copies the user's controller from {@link #MANUAL_SOURCE_PORT} onto the robot's port as an Xbox layout. */
  private void relayManualController() {
    int src = MANUAL_SOURCE_PORT;
    if (!DriverStationBackend.isJoystickConnected(src)) {
      if (relaying) {
        gamepad.disconnect();
        relaying = false;
      }
      return;
    }
    double[] in = new double[6];
    int axesMask = DriverStationBackend.getStickAxesAvailable(src);
    for (int i = 0; i < in.length; i++) {
      if ((axesMask & (1 << i)) != 0) {
        in[i] = DriverStationBackend.getStickAxis(src, i);
      }
    }
    long buttons = DriverStationBackend.getStickButtons(src);
    GamepadLayout.XboxState state;
    if (DriverStationBackend.getJoystickIsGamepad(src)) {
      state = GamepadLayout.fromGamepad(in, buttons);
    } else { // "Map gamepad" off: pass the raw layout through unchanged
      int pov = (DriverStationBackend.getStickPOVsAvailable(src) & 1) != 0 ? DriverStationBackend.getStickPOV(src, 0).value : 0;
      state = new GamepadLayout.XboxState(in, (int) (buttons & 0x3FF), pov);
    }
    gamepad.name(DriverStationBackend.getJoystickName(src) + " (via Joystick " + src + ")");
    gamepad.setAll(state.axes(), state.buttons(), state.pov());
    gamepad.flush();
    relaying = true;
  }

  private boolean manualWasEnabled;
  private RobotMode manualMode;

  /**
   * Counts the DS match time down while the user has the robot enabled (20 s auto, 140 s teleop),
   * restarting whenever they enable or switch mode, so hub-shift logic behaves like a match.
   */
  private void manualPeriodic() {
    boolean enabled = DriverStationSim.getEnabled();
    RobotMode mode = DriverStationSim.getRobotMode();
    if (enabled && (!manualWasEnabled || mode != manualMode)) {
      phaseTimer.restart();
      System.out.println("[Sim] Robot ENABLED (" + mode + ")");
    } else if (!enabled && manualWasEnabled) {
      System.out.println("[Sim] Robot disabled. To drive: Robot State > select Teleoperated > click Enable.");
    }
    manualWasEnabled = enabled;
    manualMode = mode;
    if (enabled) {
      double start = mode == RobotMode.AUTONOMOUS ? Segment.DEFAULT_AUTO_DURATION : Segment.DEFAULT_TELEOP_MATCH_TIME;
      DriverStationSim.setMatchTime(Math.max(0.0, start - phaseTimer.get()));
    }
    relayManualController();
    reportManualJoystick();
  }

  private final Timer joystickReportTimer = new Timer();
  private String lastJoystickReport = "";

  /**
   * Once a second, reports where the user's controller is and what the robot gets (console when it
   * changes, and Sim/Manual/Controller), so a missing or misplaced gamepad is obvious.
   */
  private void reportManualJoystick() {
    if (joystickReportTimer.isRunning() && !joystickReportTimer.hasElapsed(1.0)) {
      return;
    }
    joystickReportTimer.restart();
    int src = MANUAL_SOURCE_PORT;
    int dst = RobotContainer.controller.getPort();
    String report;
    if (relaying) {
      report = String.format("\"%s\" on Joystick %d -> robot Joystick %d as an Xbox controller%s",
          DriverStationBackend.getJoystickName(src), src, dst,
          DriverStationBackend.getJoystickIsGamepad(src) ? ""
              : ". \"Map gamepad\" is OFF for Joystick " + src + ", so inputs are passed through unconverted; tick it");
    } else if (DriverStationBackend.isJoystickConnected(dst)) {
      report = String.format("\"%s\" is on Joystick %d, where the robot reads it in the wrong (2027) layout. In the sim GUI,"
          + " drag it to Joystick %d instead; the simulator converts it onto Joystick %d.",
          DriverStationBackend.getJoystickName(dst), dst, src, dst);
    } else {
      report = "no controller on Joystick " + src + ". In the sim GUI, drag your controller from System Joysticks onto"
          + " Joystick " + src + " and tick \"Map gamepad\".";
    }
    Logger.recordOutput("Sim/Manual/Controller", report);
    if (!report.equals(lastJoystickReport)) {
      System.out.println("[Sim] Controller: " + report);
      lastJoystickReport = report;
    }
  }

  // ================================================================== helpers

  private void finish(boolean stoppedEarly) {
    phase = Phase.DONE;
    DriverStationSim.setEnabled(false);
    results.finish(paths.latestResults(), stoppedEarly);
    System.exit(results.passed() ? 0 : 1);
  }

  private void setPose(ScenarioEvent.SetPose event) {
    Pose pose = field.resolve(event.location(), event.pose(), event.frame(), test.alliance());
    if (pose == null) {
      results.testError("setPose: unknown location " + event.location());
      return;
    }
    setRobotPose(pose);
  }

  private void setRobotPose(Pose pose) {
    Pose2d p = new Pose2d(pose.x(), pose.y(), Rotation2d.fromDegrees(pose.deg()));
    RobotContainer.drive.setPose(p);
    RobotContainer.visionGlobalPose.setRobotPose(p);
  }

  /** Logs controller edges to Sim/Debug. Bindings are permanent, so they're added only once. */
  private void registerDebugBindings() {
    if (debugBindingsRegistered) {
      return;
    }
    debugBindingsRegistered = true;
    var c = RobotContainer.commandController;
    Map<String, Trigger> inputs = Map.of(
        "Button X", c.x(),
        "Left Bumper", c.leftBumper(),
        "Right Bumper", c.rightBumper(),
        "Left Trigger", c.leftTrigger(),
        "Right Trigger", c.rightTrigger());
    inputs.forEach((name, trigger) -> {
      trigger.onTrue(Commands.runOnce(() -> Logger.recordOutput("Sim/Debug", name + " True")));
      trigger.onFalse(Commands.runOnce(() -> Logger.recordOutput("Sim/Debug", name + " False")));
    });
  }
}
