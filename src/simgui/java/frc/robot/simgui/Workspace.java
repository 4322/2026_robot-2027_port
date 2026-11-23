package frc.robot.simgui;

import frc.robot.autonomous.AutonomousSelector.AutoName;
import frc.robot.sim.scenario.FieldLocations;
import frc.robot.sim.scenario.ProbeCatalog;
import frc.robot.sim.scenario.Scenario;
import frc.robot.sim.scenario.ScenarioEvent;
import frc.robot.sim.scenario.ScenarioJson;
import frc.robot.sim.scenario.Segment;
import frc.robot.sim.scenario.SimPaths;
import frc.robot.sim.scenario.Suite;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.stream.Stream;

/**
 * Everything the editor has loaded from {@code simulation/}: field locations, scenarios and
 * suites, with dirty tracking. All GUI panels share one Workspace and listen for changes.
 */
final class Workspace {
  final SimPaths paths;
  private FieldLocations field = new FieldLocations();
  private boolean fieldDirty;
  private final Map<String, Scenario> scenarios = new TreeMap<>();
  private final Set<String> dirtyScenarios = new TreeSet<>();
  private final Map<String, Suite> suites = new TreeMap<>();
  private final Set<String> dirtySuites = new TreeSet<>();
  private final List<Runnable> listeners = new CopyOnWriteArrayList<>();
  /** Problems hit while loading (bad JSON etc.), shown once at startup. */
  final List<String> loadErrors = new ArrayList<>();

  Workspace(SimPaths paths) {
    this.paths = paths;
    reload();
  }

  // ------------------------------------------------------------------ loading

  void reload() {
    loadErrors.clear();
    scenarios.clear();
    suites.clear();
    dirtyScenarios.clear();
    dirtySuites.clear();
    fieldDirty = false;
    try {
      field = ScenarioJson.readFieldLocations(paths.fieldLocations());
    } catch (IOException e) {
      loadErrors.add(e.getMessage());
      field = new FieldLocations();
    }
    for (Path p : jsonFiles(paths.scenariosDir())) {
      try {
        scenarios.put(ScenarioJson.idOf(p), ScenarioJson.readScenario(p));
      } catch (IOException e) {
        loadErrors.add(e.getMessage());
      }
    }
    for (Path p : jsonFiles(paths.suitesDir())) {
      try {
        suites.put(ScenarioJson.idOf(p), ScenarioJson.readSuite(p));
      } catch (IOException e) {
        loadErrors.add(e.getMessage());
      }
    }
    fireChanged();
  }

  private static List<Path> jsonFiles(Path dir) {
    if (!Files.isDirectory(dir)) {
      return List.of();
    }
    try (Stream<Path> s = Files.list(dir)) {
      return s.filter(p -> p.getFileName().toString().endsWith(".json")).sorted().toList();
    } catch (IOException e) {
      return List.of();
    }
  }

  // ------------------------------------------------------------------ change notification

  void addListener(Runnable r) {
    listeners.add(r);
  }

  void fireChanged() {
    for (Runnable r : listeners) {
      r.run();
    }
  }

  // ------------------------------------------------------------------ field locations

  FieldLocations field() {
    return field;
  }

  void markFieldDirty() {
    fieldDirty = true;
    fireChanged();
  }

  boolean isFieldDirty() {
    return fieldDirty;
  }

  void saveField() throws IOException {
    ScenarioJson.writeFieldLocations(paths.fieldLocations(), field);
    fieldDirty = false;
    fireChanged();
  }

  /** Scenario ids whose events reference the named location. */
  List<String> scenariosUsingLocation(String location) {
    List<String> out = new ArrayList<>();
    for (Map.Entry<String, Scenario> e : scenarios.entrySet()) {
      if (segments(e.getValue()).anyMatch(seg -> seg.events().stream().anyMatch(ev -> location.equals(locationOf(ev))))) {
        out.add(e.getKey());
      }
    }
    return out;
  }

  /** Renames a location everywhere (field file and every scenario that uses it). */
  void renameLocation(String oldName, String newName) {
    var renamed = new java.util.LinkedHashMap<String, frc.robot.sim.scenario.Pose>();
    field.locations().forEach((k, v) -> renamed.put(k.equals(oldName) ? newName : k, v));
    field.locations().clear();
    field.locations().putAll(renamed);
    fieldDirty = true;
    for (Map.Entry<String, Scenario> e : scenarios.entrySet()) {
      boolean changed = false;
      for (Segment seg : segments(e.getValue()).toList()) {
        for (int i = 0; i < seg.events().size(); i++) {
          ScenarioEvent ev = seg.events().get(i);
          if (oldName.equals(locationOf(ev))) {
            seg.events().set(i, withLocation(ev, newName));
            changed = true;
          }
        }
      }
      if (changed) {
        dirtyScenarios.add(e.getKey());
      }
    }
    fireChanged();
  }

  static String locationOf(ScenarioEvent ev) {
    return switch (ev) {
      case ScenarioEvent.SetPose p -> p.location();
      case ScenarioEvent.Expect x -> x.location();
      default -> null;
    };
  }

  private static ScenarioEvent withLocation(ScenarioEvent ev, String loc) {
    return switch (ev) {
      case ScenarioEvent.SetPose p -> new ScenarioEvent.SetPose(p.waitSec(), p.label(), loc, p.pose(), p.frame());
      case ScenarioEvent.Expect x -> new ScenarioEvent.Expect(
          x.waitSec(), x.label(), x.probe(), x.equalsValue(), x.min(), x.max(), loc, x.pose(), x.frame(),
          x.tolerance(), x.headingTolDeg(), x.within());
      default -> ev;
    };
  }

  private static Stream<Segment> segments(Scenario s) {
    return Stream.of(s.getAuto(), s.getTeleop()).filter(java.util.Objects::nonNull);
  }

  // ------------------------------------------------------------------ scenarios

  Set<String> scenarioIds() {
    return scenarios.keySet();
  }

  Scenario scenario(String id) {
    return scenarios.get(id);
  }

  boolean isDirty(String scenarioId) {
    return dirtyScenarios.contains(scenarioId);
  }

  void markDirty(String scenarioId) {
    dirtyScenarios.add(scenarioId);
    fireChanged();
  }

  List<String> validate(Scenario s) {
    return ScenarioJson.validate(s, field, autoNames(), ProbeCatalog.names());
  }

  /** Saves one scenario. Returns validation problems instead of saving if it's invalid. */
  List<String> saveScenario(String id) throws IOException {
    Scenario s = scenarios.get(id);
    List<String> problems = validate(s);
    if (!problems.isEmpty()) {
      return problems;
    }
    ScenarioJson.writeScenario(paths.scenario(id), s);
    dirtyScenarios.remove(id);
    fireChanged();
    return List.of();
  }

  /** Re-reads one scenario from disk, discarding edits. */
  void revertScenario(String id) throws IOException {
    Path p = paths.scenario(id);
    if (Files.exists(p)) {
      scenarios.put(id, ScenarioJson.readScenario(p));
    } else {
      scenarios.remove(id);
    }
    dirtyScenarios.remove(id);
    fireChanged();
  }

  /** Adds a new, unsaved scenario. */
  void addScenario(String id, Scenario s) {
    scenarios.put(id, s);
    dirtyScenarios.add(id);
    fireChanged();
  }

  void deleteScenario(String id) throws IOException {
    Files.deleteIfExists(paths.scenario(id));
    scenarios.remove(id);
    dirtyScenarios.remove(id);
    fireChanged();
  }

  static boolean isValidId(String id) {
    return id != null && id.matches("[a-z0-9][a-z0-9_-]*");
  }

  // ------------------------------------------------------------------ suites

  Set<String> suiteIds() {
    return suites.keySet();
  }

  Suite suite(String id) {
    return suites.get(id);
  }

  void putSuite(String id, Suite s) {
    suites.put(id, s);
    dirtySuites.add(id);
    fireChanged();
  }

  void markSuiteDirty(String id) {
    dirtySuites.add(id);
    fireChanged();
  }

  boolean isSuiteDirty(String id) {
    return dirtySuites.contains(id);
  }

  void saveSuite(String id) throws IOException {
    ScenarioJson.writeSuite(paths.suite(id), suites.get(id));
    dirtySuites.remove(id);
    fireChanged();
  }

  void deleteSuite(String id) throws IOException {
    Files.deleteIfExists(paths.suite(id));
    suites.remove(id);
    dirtySuites.remove(id);
    fireChanged();
  }

  // ------------------------------------------------------------------ everything

  boolean hasUnsavedChanges() {
    return fieldDirty || !dirtyScenarios.isEmpty() || !dirtySuites.isEmpty();
  }

  /** Saves every dirty file. Returns problems for files that couldn't be saved. */
  List<String> saveAll() {
    List<String> problems = new ArrayList<>();
    try {
      if (fieldDirty) {
        saveField();
      }
    } catch (IOException e) {
      problems.add("field-locations.json: " + e.getMessage());
    }
    for (String id : new ArrayList<>(dirtyScenarios)) {
      try {
        List<String> p = saveScenario(id);
        p.forEach(msg -> problems.add(id + ": " + msg));
      } catch (IOException e) {
        problems.add(id + ": " + e.getMessage());
      }
    }
    for (String id : new ArrayList<>(dirtySuites)) {
      try {
        saveSuite(id);
      } catch (IOException e) {
        problems.add("suite " + id + ": " + e.getMessage());
      }
    }
    return problems;
  }

  /** Every AutoName constant (the editor's auto routine choices). */
  static List<String> autoNames() {
    return Arrays.stream(AutoName.values()).map(Enum::name).toList();
  }

  /** Location names, in file order. */
  List<String> locationNames() {
    return new ArrayList<>(new LinkedHashSet<>(field.locations().keySet()));
  }
}
