package frc.robot.sim.scenario;

import frc.robot.sim.scenario.ScenarioEvent.ButtonAction;
import frc.robot.sim.scenario.ScenarioEvent.Frame;
import frc.robot.sim.scenario.ScenarioEvent.GamepadButton;
import frc.robot.sim.scenario.ScenarioEvent.PovDirection;
import frc.robot.sim.scenario.ScenarioEvent.TriggerSide;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardCopyOption;
import java.util.ArrayList;
import java.util.Collection;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Set;
import org.json.simple.parser.ContainerFactory;
import org.json.simple.parser.JSONParser;
import org.json.simple.parser.ParseException;

/**
 * Reads, writes and validates every simulator JSON file (scenarios, field locations, suites, run
 * config, run report).
 *
 * <p>Parsing is strict: unknown event types, bad enum names and wrong value types throw a {@link
 * FormatException} naming the JSON path (e.g. {@code teleop.events[3].button}). Semantic checks
 * that need context (unknown location names, value ranges, ...) are in {@link #validate}.
 *
 * <p>Files are written pretty-printed with each event on one line so diffs stay readable.
 */
public final class ScenarioJson {
  private ScenarioJson() {}

  /** A JSON file that couldn't be understood. The message starts with the JSON path. */
  public static class FormatException extends IOException {
    public FormatException(String message) {
      super(message);
    }
  }

  // =================================================================== file helpers

  /** File id used to reference scenarios/suites: the file name without ".json". */
  public static String idOf(Path file) {
    String n = file.getFileName().toString();
    return n.endsWith(".json") ? n.substring(0, n.length() - 5) : n;
  }

  public static Scenario readScenario(Path file) throws IOException {
    return withFile(file, () -> parseScenario(parse(Files.readString(file, StandardCharsets.UTF_8))));
  }

  public static void writeScenario(Path file, Scenario s) throws IOException {
    writeAtomically(file, toJson(scenarioToMap(s)));
  }

  public static FieldLocations readFieldLocations(Path file) throws IOException {
    return withFile(file, () -> parseFieldLocations(parse(Files.readString(file, StandardCharsets.UTF_8))));
  }

  public static void writeFieldLocations(Path file, FieldLocations f) throws IOException {
    writeAtomically(file, toJson(fieldLocationsToMap(f)));
  }

  public static Suite readSuite(Path file) throws IOException {
    return withFile(file, () -> parseSuite(parse(Files.readString(file, StandardCharsets.UTF_8))));
  }

  public static void writeSuite(Path file, Suite s) throws IOException {
    writeAtomically(file, toJson(suiteToMap(s)));
  }

  public static RunConfig readRunConfig(Path file) throws IOException {
    return withFile(file, () -> parseRunConfig(parse(Files.readString(file, StandardCharsets.UTF_8))));
  }

  public static void writeRunConfig(Path file, RunConfig c) throws IOException {
    writeAtomically(file, toJson(runConfigToMap(c)));
  }

  public static RunReport readRunReport(Path file) throws IOException {
    return withFile(file, () -> parseRunReport(parse(Files.readString(file, StandardCharsets.UTF_8))));
  }

  public static void writeRunReport(Path file, RunReport r) throws IOException {
    writeAtomically(file, toJson(runReportToMap(r)));
  }

  private interface IoSupplier<T> {
    T get() throws IOException;
  }

  /** Prefixes format errors with the file name. */
  private static <T> T withFile(Path file, IoSupplier<T> body) throws IOException {
    try {
      return body.get();
    } catch (FormatException e) {
      throw new FormatException(file.getFileName() + ": " + e.getMessage());
    }
  }

  /** Writes via a temp file + move so readers (sim / GUI) never see a half-written file. */
  private static void writeAtomically(Path file, String content) throws IOException {
    Path dir = file.toAbsolutePath().getParent();
    Files.createDirectories(dir);
    Path tmp = Files.createTempFile(dir, ".tmp-", ".json");
    try {
      Files.writeString(tmp, content, StandardCharsets.UTF_8);
      try {
        Files.move(tmp, file, StandardCopyOption.REPLACE_EXISTING, StandardCopyOption.ATOMIC_MOVE);
      } catch (IOException atomicNotSupported) {
        Files.move(tmp, file, StandardCopyOption.REPLACE_EXISTING);
      }
    } finally {
      Files.deleteIfExists(tmp);
    }
  }

  // =================================================================== scenario

  public static Scenario parseScenario(Object root) throws FormatException {
    Obj o = Obj.of(root, "");
    Scenario s = new Scenario(o.str("name"));
    s.setDescription(o.strOr("description", ""));
    s.setGameData(o.strOr("gameData", ""));
    s.setDebugBindings(o.boolOr("debugBindings", false));
    Obj auto = o.objOrNull("auto");
    if (auto != null) {
      Segment seg = new Segment(true);
      seg.setAutoRoutine(auto.str("routine"));
      seg.setDurationSec(auto.numOrNull("duration"));
      seg.setStartMatchTime(auto.numOrNull("startMatchTime"));
      seg.events().addAll(parseEvents(auto));
      s.setAuto(seg);
    }
    Obj teleop = o.objOrNull("teleop");
    if (teleop != null) {
      Segment seg = new Segment(false);
      seg.setDurationSec(teleop.numOrNull("duration"));
      seg.setStartMatchTime(teleop.numOrNull("startMatchTime"));
      seg.events().addAll(parseEvents(teleop));
      s.setTeleop(seg);
    }
    return s;
  }

  private static List<ScenarioEvent> parseEvents(Obj segment) throws FormatException {
    List<ScenarioEvent> events = new ArrayList<>();
    List<Obj> list = segment.objListOrEmpty("events");
    for (Obj e : list) {
      events.add(parseEvent(e));
    }
    return events;
  }

  public static ScenarioEvent parseEvent(Obj e) throws FormatException {
    double wait = e.numOr("wait", 0.0);
    String label = e.strOr("label", null);
    String type = e.str("type");
    return switch (type) {
      case "setPose" -> new ScenarioEvent.SetPose(
          wait, label, e.strOr("location", null), e.poseOrNull(), e.enumOr("frame", Frame.class, Frame.ALLIANCE));
      case "button" -> new ScenarioEvent.Button(
          wait, label, e.enumReq("button", GamepadButton.class), e.enumReq("action", ButtonAction.class));
      case "pov" -> new ScenarioEvent.Pov(
          wait, label, e.enumReq("direction", PovDirection.class), e.enumOr("action", ButtonAction.class, ButtonAction.HOLD));
      case "trigger" -> new ScenarioEvent.Trigger(
          wait, label, e.enumReq("side", TriggerSide.class), e.num("value"));
      case "drive" -> new ScenarioEvent.Drive(wait, label, e.num("x"), e.num("y"));
      case "turn" -> new ScenarioEvent.Turn(wait, label, e.num("rate"));
      case "stopJoysticks" -> new ScenarioEvent.StopJoysticks(wait, label);
      case "gameData" -> new ScenarioEvent.GameData(wait, label, e.strOr("value", ""));
      case "expect" -> new ScenarioEvent.Expect(
          wait,
          label,
          e.str("probe"),
          e.scalarAsStringOrNull("equals"),
          e.numOrNull("min"),
          e.numOrNull("max"),
          e.strOr("location", null),
          e.poseOrNull(),
          e.enumOr("frame", Frame.class, Frame.ALLIANCE),
          e.numOrNull("tolerance"),
          e.numOrNull("headingTolDeg"),
          e.numOrNull("within"));
      case "end" -> new ScenarioEvent.End(wait, label);
      default -> throw e.error("type", "unknown event type \"" + type + "\" (expected one of " + EVENT_TYPES + ")");
    };
  }

  /** Every valid {@code "type"} value, in the order the editor offers them. */
  public static final List<String> EVENT_TYPES =
      List.of("setPose", "drive", "turn", "stopJoysticks", "button", "pov", "trigger", "gameData", "expect", "end");

  public static Map<String, Object> scenarioToMap(Scenario s) {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("name", s.getName());
    if (!s.getDescription().isEmpty()) {
      m.put("description", s.getDescription());
    }
    if (!s.getGameData().isEmpty()) {
      m.put("gameData", s.getGameData());
    }
    if (s.isDebugBindings()) {
      m.put("debugBindings", true);
    }
    if (s.getAuto() != null) {
      Segment a = s.getAuto();
      Map<String, Object> am = new LinkedHashMap<>();
      am.put("routine", a.getAutoRoutine());
      putIfNotNull(am, "duration", a.getDurationSec());
      putIfNotNull(am, "startMatchTime", a.getStartMatchTime());
      am.put("events", eventsToList(a.events()));
      m.put("auto", am);
    }
    if (s.getTeleop() != null) {
      Segment t = s.getTeleop();
      Map<String, Object> tm = new LinkedHashMap<>();
      putIfNotNull(tm, "duration", t.getDurationSec());
      putIfNotNull(tm, "startMatchTime", t.getStartMatchTime());
      tm.put("events", eventsToList(t.events()));
      m.put("teleop", tm);
    }
    return m;
  }

  private static List<Object> eventsToList(List<ScenarioEvent> events) {
    List<Object> list = new ArrayList<>();
    for (ScenarioEvent e : events) {
      list.add(eventToMap(e));
    }
    return list;
  }

  public static Map<String, Object> eventToMap(ScenarioEvent e) {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("wait", e.waitSec());
    m.put("type", e.type());
    switch (e) {
      case ScenarioEvent.SetPose p -> {
        putIfNotNull(m, "location", p.location());
        putPose(m, p.pose());
        putFrame(m, p.frame());
      }
      case ScenarioEvent.Button b -> {
        m.put("button", b.button().name());
        m.put("action", lower(b.action()));
      }
      case ScenarioEvent.Pov p -> {
        m.put("direction", p.direction().name());
        m.put("action", lower(p.action()));
      }
      case ScenarioEvent.Trigger t -> {
        m.put("side", t.side().name());
        m.put("value", t.value());
      }
      case ScenarioEvent.Drive d -> {
        m.put("x", d.x());
        m.put("y", d.y());
      }
      case ScenarioEvent.Turn t -> m.put("rate", t.rate());
      case ScenarioEvent.StopJoysticks s -> {}
      case ScenarioEvent.GameData g -> m.put("value", g.value());
      case ScenarioEvent.Expect x -> {
        m.put("probe", x.probe());
        putIfNotNull(m, "equals", x.equalsValue());
        putIfNotNull(m, "min", x.min());
        putIfNotNull(m, "max", x.max());
        putIfNotNull(m, "location", x.location());
        putPose(m, x.pose());
        if (x.isPoseCheck()) {
          putFrame(m, x.frame());
        }
        putIfNotNull(m, "tolerance", x.tolerance());
        putIfNotNull(m, "headingTolDeg", x.headingTolDeg());
        putIfNotNull(m, "within", x.within());
      }
      case ScenarioEvent.End end -> {}
    }
    putIfNotNull(m, "label", e.label());
    return m;
  }

  private static void putPose(Map<String, Object> m, Pose p) {
    if (p != null) {
      m.put("x", p.x());
      m.put("y", p.y());
      m.put("deg", p.deg());
    }
  }

  private static void putFrame(Map<String, Object> m, Frame f) {
    if (f == Frame.ABSOLUTE) {
      m.put("frame", "absolute");
    }
  }

  // =================================================================== validation

  /**
   * Semantic checks on a scenario. Returns human-readable problems; empty means valid.
   *
   * @param locations field locations, for checking location names (may be null to skip)
   * @param knownAutos valid auto routine names (may be null to skip)
   * @param knownProbes valid expect probe names (may be null to skip)
   */
  public static List<String> validate(
      Scenario s, FieldLocations locations, Collection<String> knownAutos, Collection<String> knownProbes) {
    List<String> problems = new ArrayList<>();
    if (s.getName() == null || s.getName().isBlank()) {
      problems.add("name: must not be empty");
    }
    if (s.getAuto() == null && s.getTeleop() == null) {
      problems.add("scenario has neither an auto nor a teleop segment");
    }
    if (!Set.of("", "B", "R").contains(s.getGameData())) {
      problems.add("gameData: must be \"\", \"B\" or \"R\"");
    }
    if (s.getAuto() != null) {
      String r = s.getAuto().getAutoRoutine();
      if (r == null || r.isBlank()) {
        problems.add("auto.routine: must not be empty");
      } else if (knownAutos != null && !knownAutos.contains(r)) {
        problems.add("auto.routine: unknown auto \"" + r + "\"");
      }
      validateSegment("auto", s.getAuto(), locations, knownProbes, problems);
    }
    if (s.getTeleop() != null) {
      if (s.getTeleop().events().isEmpty() && s.getTeleop().getDurationSec() == null) {
        problems.add("teleop: has no events and no duration, so it would end immediately");
      }
      validateSegment("teleop", s.getTeleop(), locations, knownProbes, problems);
    }
    return problems;
  }

  private static void validateSegment(
      String name, Segment seg, FieldLocations locations, Collection<String> knownProbes, List<String> problems) {
    if (seg.getDurationSec() != null && seg.getDurationSec() < 0) {
      problems.add(name + ".duration: must be >= 0");
    }
    if (seg.getStartMatchTime() != null && seg.getStartMatchTime() < 0) {
      problems.add(name + ".startMatchTime: must be >= 0");
    }
    List<ScenarioEvent> events = seg.events();
    for (int i = 0; i < events.size(); i++) {
      ScenarioEvent e = events.get(i);
      String p = name + ".events[" + i + "] (" + e.displayName() + ")";
      if (!(e.waitSec() >= 0) || Double.isInfinite(e.waitSec())) {
        problems.add(p + ": wait must be a number >= 0");
      }
      switch (e) {
        case ScenarioEvent.SetPose sp -> checkPoseRef(p, sp.location(), sp.pose(), locations, problems);
        case ScenarioEvent.Trigger t -> checkRange(p, "value", t.value(), 0, 1, problems);
        case ScenarioEvent.Drive d -> {
          checkRange(p, "x", d.x(), -1, 1, problems);
          checkRange(p, "y", d.y(), -1, 1, problems);
          if (Math.hypot(d.x(), d.y()) > 1.0 + 1e-9) {
            problems.add(p + ": stick vector length must be <= 1");
          }
        }
        case ScenarioEvent.Turn t -> checkRange(p, "rate", t.rate(), -1, 1, problems);
        case ScenarioEvent.GameData g -> {
          if (!Set.of("", "B", "R").contains(g.value())) {
            problems.add(p + ": value must be \"\", \"B\" or \"R\"");
          }
        }
        case ScenarioEvent.Expect x -> validateExpect(p, x, locations, knownProbes, problems);
        case ScenarioEvent.End end -> {
          if (i != events.size() - 1) {
            problems.add(p + ": \"end\" must be the last event");
          }
        }
        default -> {}
      }
    }
  }

  private static void validateExpect(
      String p, ScenarioEvent.Expect x, FieldLocations locations, Collection<String> knownProbes, List<String> problems) {
    if (x.probe() == null || x.probe().isBlank()) {
      problems.add(p + ": probe must not be empty");
      return;
    }
    if (knownProbes != null && !knownProbes.contains(x.probe())) {
      problems.add(p + ": unknown probe \"" + x.probe() + "\"");
    }
    if (x.isPoseCheck()) {
      checkPoseRef(p, x.location(), x.pose(), locations, problems);
      if (x.tolerance() != null && x.tolerance() <= 0) {
        problems.add(p + ": tolerance must be > 0");
      }
      if (x.headingTolDeg() != null && x.headingTolDeg() <= 0) {
        problems.add(p + ": headingTolDeg must be > 0");
      }
    } else {
      if (x.equalsValue() == null && x.min() == null && x.max() == null) {
        problems.add(p + ": needs \"equals\", \"min\" or \"max\"");
      }
      if (x.min() != null && x.max() != null && x.min() > x.max()) {
        problems.add(p + ": min is greater than max");
      }
    }
    if (x.within() != null && x.within() < 0) {
      problems.add(p + ": within must be >= 0");
    }
  }

  private static void checkPoseRef(
      String p, String location, Pose pose, FieldLocations locations, List<String> problems) {
    if (location == null && pose == null) {
      problems.add(p + ": needs a \"location\" or x/y/deg");
    } else if (location != null && pose != null) {
      problems.add(p + ": give either a \"location\" or x/y/deg, not both");
    } else if (location != null && locations != null && !locations.locations().containsKey(location)) {
      problems.add(p + ": unknown location \"" + location + "\"");
    }
  }

  private static void checkRange(String p, String field, double v, double lo, double hi, List<String> problems) {
    if (!(v >= lo && v <= hi)) {
      problems.add(p + ": " + field + " must be between " + ScenarioEvent.fmt(lo) + " and " + ScenarioEvent.fmt(hi));
    }
  }

  // =================================================================== field locations

  public static FieldLocations parseFieldLocations(Object root) throws FormatException {
    Obj o = Obj.of(root, "");
    FieldLocations f = new FieldLocations();
    Obj field = o.objOrNull("field");
    if (field != null) {
      f.setFieldLength(field.num("length"));
      f.setFieldWidth(field.num("width"));
    }
    Obj preview = o.objOrNull("preview");
    if (preview != null) {
      f.setPreviewMaxSpeedMps(preview.numOr("maxSpeedMps", f.getPreviewMaxSpeedMps()));
      f.setPreviewMaxTurnDegPerSec(preview.numOr("maxTurnDegPerSec", f.getPreviewMaxTurnDegPerSec()));
    }
    Obj locs = o.objOrNull("locations");
    if (locs != null) {
      for (String name : locs.keys()) {
        Obj l = locs.obj(name);
        f.locations().put(name, new Pose(l.num("x"), l.num("y"), l.numOr("deg", 0.0)));
      }
    }
    for (Obj ft : o.objListOrEmpty("features")) {
      f.features().add(new FieldLocations.Feature(
          ft.str("name"), ft.num("x1"), ft.num("y1"), ft.num("x2"), ft.num("y2"), ft.strOr("color", null)));
    }
    return f;
  }

  public static Map<String, Object> fieldLocationsToMap(FieldLocations f) {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("field", mapOf("length", f.getFieldLength(), "width", f.getFieldWidth()));
    m.put("preview", mapOf("maxSpeedMps", f.getPreviewMaxSpeedMps(), "maxTurnDegPerSec", f.getPreviewMaxTurnDegPerSec()));
    Map<String, Object> locs = new LinkedHashMap<>();
    for (Map.Entry<String, Pose> e : f.locations().entrySet()) {
      Pose p = e.getValue();
      locs.put(e.getKey(), mapOf("x", p.x(), "y", p.y(), "deg", p.deg()));
    }
    m.put("locations", locs);
    List<Object> features = new ArrayList<>();
    for (FieldLocations.Feature ft : f.features()) {
      Map<String, Object> fm = mapOf("name", ft.name(), "x1", ft.x1(), "y1", ft.y1(), "x2", ft.x2(), "y2", ft.y2());
      putIfNotNull(fm, "color", ft.color());
      features.add(fm);
    }
    m.put("features", features);
    return m;
  }

  // =================================================================== suites

  public static Suite parseSuite(Object root) throws FormatException {
    Obj o = Obj.of(root, "");
    Suite s = new Suite(o.str("name"));
    for (Obj r : o.objListOrEmpty("runs")) {
      String autos = r.strOr("autos", null);
      boolean all = autos != null;
      if (all && !autos.equals("ALL")) {
        throw r.error("autos", "only \"ALL\" is supported");
      }
      String scenario = r.strOr("scenario", null);
      if (all == (scenario != null)) {
        throw r.error("", "needs exactly one of \"scenario\" or \"autos\"");
      }
      List<SimAlliance> alliances = r.enumList("alliances", SimAlliance.class);
      if (alliances.isEmpty()) {
        throw r.error("alliances", "must list at least one alliance");
      }
      s.runs().add(new Suite.Run(scenario, all, alliances, r.numOrNull("duration")));
    }
    return s;
  }

  public static Map<String, Object> suiteToMap(Suite s) {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("name", s.getName());
    List<Object> runs = new ArrayList<>();
    for (Suite.Run r : s.runs()) {
      Map<String, Object> rm = new LinkedHashMap<>();
      if (r.allAutos()) {
        rm.put("autos", "ALL");
      } else {
        rm.put("scenario", r.scenario());
      }
      rm.put("alliances", enumNames(r.alliances()));
      putIfNotNull(rm, "duration", r.autoDuration());
      runs.add(rm);
    }
    m.put("runs", runs);
    return m;
  }

  // =================================================================== run config

  public static RunConfig parseRunConfig(Object root) throws FormatException {
    Obj o = Obj.of(root, "");
    RunConfig c = new RunConfig();
    c.setMode(o.enumOr("mode", RunConfig.Mode.class, RunConfig.Mode.SCRIPTED));
    c.setSuite(o.strOr("suite", null));
    c.scenarios().addAll(o.strList("scenarios"));
    c.alliances().addAll(o.enumList("alliances", SimAlliance.class));
    c.setManualAlliance(o.enumOr("alliance", SimAlliance.class, SimAlliance.BLUE));
    c.setManualRobotMode(o.enumOr("robotMode", RunConfig.ManualRobotMode.class, RunConfig.ManualRobotMode.TELEOPERATED));
    c.setStartLocation(o.strOr("startLocation", null));
    c.setAutoRoutine(o.strOr("autoRoutine", null));
    c.setWarmupSec(o.numOr("warmupSec", RunConfig.DEFAULT_WARMUP_SEC));
    c.setGapSec(o.numOr("gapSec", RunConfig.DEFAULT_GAP_SEC));
    if (c.getMode() == RunConfig.Mode.SCRIPTED) {
      boolean hasSuite = c.getSuite() != null;
      boolean hasList = !c.scenarios().isEmpty();
      if (hasSuite == hasList) {
        throw o.error("", "SCRIPTED mode needs exactly one of \"suite\" or \"scenarios\"");
      }
      if (hasList && c.alliances().isEmpty()) {
        c.alliances().add(SimAlliance.BLUE);
      }
    }
    return c;
  }

  public static Map<String, Object> runConfigToMap(RunConfig c) {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("mode", c.getMode().name());
    if (c.getMode() == RunConfig.Mode.SCRIPTED) {
      if (c.getSuite() != null) {
        m.put("suite", c.getSuite());
      } else {
        m.put("scenarios", new ArrayList<Object>(c.scenarios()));
        m.put("alliances", enumNames(c.alliances()));
      }
    } else {
      m.put("alliance", c.getManualAlliance().name());
      m.put("robotMode", c.getManualRobotMode().name());
      putIfNotNull(m, "startLocation", c.getStartLocation());
      putIfNotNull(m, "autoRoutine", c.getAutoRoutine());
    }
    if (c.getWarmupSec() != RunConfig.DEFAULT_WARMUP_SEC) {
      m.put("warmupSec", c.getWarmupSec());
    }
    if (c.getGapSec() != RunConfig.DEFAULT_GAP_SEC) {
      m.put("gapSec", c.getGapSec());
    }
    return m;
  }

  // =================================================================== run report

  public static RunReport parseRunReport(Object root) throws FormatException {
    Obj o = Obj.of(root, "");
    RunReport r = new RunReport();
    r.setStoppedEarly(o.boolOr("stoppedEarly", false));
    r.errors().addAll(o.strList("errors"));
    for (Obj t : o.objListOrEmpty("tests")) {
      RunReport.Test test = new RunReport.Test(
          t.str("name"), t.strOr("scenario", null), t.enumOr("alliance", SimAlliance.class, SimAlliance.BLUE));
      test.errors().addAll(t.strList("errors"));
      for (Obj c : t.objListOrEmpty("checks")) {
        test.checks().add(new RunReport.Check(
            c.str("segment"),
            (int) c.num("eventIndex"),
            c.str("label"),
            c.boolOr("passed", false),
            c.strOr("actual", ""),
            c.numOr("simTime", 0.0)));
      }
      r.tests().add(test);
    }
    return r;
  }

  public static Map<String, Object> runReportToMap(RunReport r) {
    Map<String, Object> m = new LinkedHashMap<>();
    m.put("passed", r.passed());
    m.put("stoppedEarly", r.isStoppedEarly());
    m.put("errors", new ArrayList<Object>(r.errors()));
    List<Object> tests = new ArrayList<>();
    for (RunReport.Test t : r.tests()) {
      Map<String, Object> tm = new LinkedHashMap<>();
      tm.put("name", t.name());
      putIfNotNull(tm, "scenario", t.scenarioId());
      tm.put("alliance", t.alliance().name());
      tm.put("passed", t.passed());
      tm.put("errors", new ArrayList<Object>(t.errors()));
      List<Object> checks = new ArrayList<>();
      for (RunReport.Check c : t.checks()) {
        checks.add(mapOf(
            "segment", c.segment(), "eventIndex", c.eventIndex(), "label", c.label(),
            "passed", c.passed(), "actual", c.actual(), "simTime", c.simTime()));
      }
      tm.put("checks", checks);
      tests.add(tm);
    }
    m.put("tests", tests);
    return m;
  }

  // =================================================================== parsing

  private static final ContainerFactory ORDERED = new ContainerFactory() {
    @Override
    public Map<String, Object> createObjectContainer() {
      return new LinkedHashMap<>();
    }

    @Override
    public List<Object> creatArrayContainer() {
      return new ArrayList<>();
    }
  };

  /** Parses JSON text into LinkedHashMap / ArrayList / String / Long / Double / Boolean / null. */
  public static Object parse(String text) throws FormatException {
    try {
      return new JSONParser().parse(text, ORDERED);
    } catch (ParseException e) {
      throw new FormatException("invalid JSON near character " + e.getPosition() + ": " + e);
    }
  }

  /** Typed, path-aware view of a JSON object. */
  public static final class Obj {
    private final Map<?, ?> map;
    private final String path;

    private Obj(Map<?, ?> map, String path) {
      this.map = map;
      this.path = path;
    }

    static Obj of(Object value, String path) throws FormatException {
      if (!(value instanceof Map<?, ?> m)) {
        throw new FormatException((path.isEmpty() ? "(root)" : path) + ": expected an object");
      }
      return new Obj(m, path);
    }

    FormatException error(String key, String message) {
      String p = key.isEmpty() ? path : (path.isEmpty() ? key : path + "." + key);
      return new FormatException((p.isEmpty() ? "(root)" : p) + ": " + message);
    }

    List<String> keys() {
      List<String> keys = new ArrayList<>();
      for (Object k : map.keySet()) {
        keys.add(String.valueOf(k));
      }
      return keys;
    }

    private String sub(String key) {
      return path.isEmpty() ? key : path + "." + key;
    }

    String str(String key) throws FormatException {
      Object v = map.get(key);
      if (!(v instanceof String s)) {
        throw error(key, v == null ? "is required" : "expected a string");
      }
      return s;
    }

    String strOr(String key, String fallback) throws FormatException {
      return map.get(key) == null ? fallback : str(key);
    }

    /** Accepts string, number or boolean and returns it as a string (for expect "equals"). */
    String scalarAsStringOrNull(String key) throws FormatException {
      Object v = map.get(key);
      if (v == null) {
        return null;
      }
      if (v instanceof String || v instanceof Number || v instanceof Boolean) {
        return String.valueOf(v);
      }
      throw error(key, "expected a string, number or boolean");
    }

    double num(String key) throws FormatException {
      Object v = map.get(key);
      if (!(v instanceof Number n)) {
        throw error(key, v == null ? "is required" : "expected a number");
      }
      return n.doubleValue();
    }

    double numOr(String key, double fallback) throws FormatException {
      return map.get(key) == null ? fallback : num(key);
    }

    Double numOrNull(String key) throws FormatException {
      return map.get(key) == null ? null : num(key);
    }

    boolean boolOr(String key, boolean fallback) throws FormatException {
      Object v = map.get(key);
      if (v == null) {
        return fallback;
      }
      if (!(v instanceof Boolean b)) {
        throw error(key, "expected true or false");
      }
      return b;
    }

    Obj obj(String key) throws FormatException {
      Object v = map.get(key);
      if (v == null) {
        throw error(key, "is required");
      }
      return Obj.of(v, sub(key));
    }

    Obj objOrNull(String key) throws FormatException {
      return map.get(key) == null ? null : obj(key);
    }

    private List<?> list(String key) throws FormatException {
      Object v = map.get(key);
      if (v == null) {
        return List.of();
      }
      if (!(v instanceof List<?> l)) {
        throw error(key, "expected an array");
      }
      return l;
    }

    List<Obj> objListOrEmpty(String key) throws FormatException {
      List<?> l = list(key);
      List<Obj> out = new ArrayList<>();
      for (int i = 0; i < l.size(); i++) {
        out.add(Obj.of(l.get(i), sub(key) + "[" + i + "]"));
      }
      return out;
    }

    List<String> strList(String key) throws FormatException {
      List<String> out = new ArrayList<>();
      List<?> l = list(key);
      for (int i = 0; i < l.size(); i++) {
        if (!(l.get(i) instanceof String s)) {
          throw error(key + "[" + i + "]", "expected a string");
        }
        out.add(s);
      }
      return out;
    }

    <E extends Enum<E>> List<E> enumList(String key, Class<E> type) throws FormatException {
      List<E> out = new ArrayList<>();
      List<String> names = strList(key);
      for (int i = 0; i < names.size(); i++) {
        out.add(toEnum(key + "[" + i + "]", names.get(i), type));
      }
      return out;
    }

    <E extends Enum<E>> E enumReq(String key, Class<E> type) throws FormatException {
      return toEnum(key, str(key), type);
    }

    <E extends Enum<E>> E enumOr(String key, Class<E> type, E fallback) throws FormatException {
      return map.get(key) == null ? fallback : enumReq(key, type);
    }

    private <E extends Enum<E>> E toEnum(String key, String name, Class<E> type) throws FormatException {
      try {
        return Enum.valueOf(type, name.trim().toUpperCase(Locale.ROOT));
      } catch (IllegalArgumentException e) {
        List<String> allowed = new ArrayList<>();
        for (E c : type.getEnumConstants()) {
          allowed.add(c.name());
        }
        throw error(key, "\"" + name + "\" is not one of " + allowed);
      }
    }

    /** Reads x/y/deg on this object as a pose, or null if none of them are present. */
    Pose poseOrNull() throws FormatException {
      if (map.get("x") == null && map.get("y") == null && map.get("deg") == null) {
        return null;
      }
      return new Pose(num("x"), num("y"), numOr("deg", 0.0));
    }
  }

  // =================================================================== writing

  /** Pretty-prints a value tree. Objects/arrays with only scalars that fit on a line stay inline. */
  public static String toJson(Object value) {
    StringBuilder sb = new StringBuilder();
    write(sb, value, 0);
    sb.append('\n');
    return sb.toString();
  }

  private static final int INLINE_LIMIT = 160;

  private static void write(StringBuilder sb, Object v, int indent) {
    if (v instanceof Map<?, ?> m) {
      if (m.isEmpty()) {
        sb.append("{}");
        return;
      }
      String inline = inline(v);
      if (inline != null && inline.length() + indent <= INLINE_LIMIT) {
        sb.append(inline);
        return;
      }
      sb.append("{\n");
      int i = 0;
      for (Map.Entry<?, ?> e : m.entrySet()) {
        pad(sb, indent + 2);
        sb.append(quote(String.valueOf(e.getKey()))).append(": ");
        write(sb, e.getValue(), indent + 2);
        sb.append(++i < m.size() ? ",\n" : "\n");
      }
      pad(sb, indent);
      sb.append('}');
    } else if (v instanceof List<?> l) {
      if (l.isEmpty()) {
        sb.append("[]");
        return;
      }
      String inline = inline(v);
      if (inline != null && inline.length() + indent <= INLINE_LIMIT) {
        sb.append(inline);
        return;
      }
      sb.append("[\n");
      for (int i = 0; i < l.size(); i++) {
        pad(sb, indent + 2);
        write(sb, l.get(i), indent + 2);
        sb.append(i + 1 < l.size() ? ",\n" : "\n");
      }
      pad(sb, indent);
      sb.append(']');
    } else {
      sb.append(scalar(v));
    }
  }

  /**
   * Single-line form of a value. Containers nest inline only when their children are scalars or
   * scalar-only arrays (so an event or suite row fits on one line, but a whole segment doesn't).
   */
  private static String inline(Object v) {
    return inline(v, 0);
  }

  private static String inline(Object v, int depth) {
    if (!(v instanceof Map) && !(v instanceof List)) {
      return scalar(v);
    }
    if (depth > 1 || (depth == 1 && v instanceof Map)) {
      return null;
    }
    StringBuilder sb = new StringBuilder();
    if (v instanceof Map<?, ?> m) {
      if (m.isEmpty()) {
        return "{}";
      }
      sb.append("{ ");
      int i = 0;
      for (Map.Entry<?, ?> e : m.entrySet()) {
        String child = inline(e.getValue(), depth + 1);
        if (child == null) {
          return null;
        }
        sb.append(quote(String.valueOf(e.getKey()))).append(": ").append(child);
        sb.append(++i < m.size() ? ", " : " ");
      }
      return sb.append('}').toString();
    }
    List<?> l = (List<?>) v;
    sb.append('[');
    for (int i = 0; i < l.size(); i++) {
      String child = inline(l.get(i), depth + 1);
      if (child == null) {
        return null;
      }
      sb.append(child);
      if (i + 1 < l.size()) {
        sb.append(", ");
      }
    }
    return sb.append(']').toString();
  }

  private static String scalar(Object v) {
    if (v == null) {
      return "null";
    }
    if (v instanceof String s) {
      return quote(s);
    }
    if (v instanceof Double || v instanceof Float) {
      double d = ((Number) v).doubleValue();
      if (Double.isNaN(d) || Double.isInfinite(d)) {
        return "null";
      }
      // Up to 4 decimals (0.1 mm), no trailing zeros, whole numbers without ".0".
      String s = java.math.BigDecimal.valueOf(d)
          .setScale(4, java.math.RoundingMode.HALF_UP)
          .stripTrailingZeros()
          .toPlainString();
      return s.equals("-0") ? "0" : s;
    }
    return String.valueOf(v);
  }

  private static String quote(String s) {
    StringBuilder sb = new StringBuilder("\"");
    for (char c : s.toCharArray()) {
      switch (c) {
        case '"' -> sb.append("\\\"");
        case '\\' -> sb.append("\\\\");
        case '\n' -> sb.append("\\n");
        case '\r' -> sb.append("\\r");
        case '\t' -> sb.append("\\t");
        default -> {
          if (c < 0x20) {
            sb.append(String.format("\\u%04x", (int) c));
          } else {
            sb.append(c);
          }
        }
      }
    }
    return sb.append('"').toString();
  }

  private static void pad(StringBuilder sb, int n) {
    sb.append(" ".repeat(n));
  }

  // =================================================================== small helpers

  private static void putIfNotNull(Map<String, Object> m, String key, Object value) {
    if (value != null) {
      m.put(key, value);
    }
  }

  private static Map<String, Object> mapOf(Object... kv) {
    Map<String, Object> m = new LinkedHashMap<>();
    for (int i = 0; i < kv.length; i += 2) {
      m.put((String) kv[i], kv[i + 1]);
    }
    return m;
  }

  private static List<Object> enumNames(Collection<? extends Enum<?>> values) {
    List<Object> out = new ArrayList<>();
    for (Enum<?> e : values) {
      out.add(e.name());
    }
    return out;
  }

  private static String lower(Enum<?> e) {
    return e.name().toLowerCase(Locale.ROOT);
  }
}
