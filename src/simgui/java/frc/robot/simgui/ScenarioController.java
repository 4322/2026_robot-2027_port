package frc.robot.simgui;

import frc.robot.sim.scenario.FieldLocations;
import frc.robot.sim.scenario.Pose;
import frc.robot.sim.scenario.ProbeCatalog;
import frc.robot.sim.scenario.Scenario;
import frc.robot.sim.scenario.ScenarioEvent;
import frc.robot.sim.scenario.ScenarioEvent.ButtonAction;
import frc.robot.sim.scenario.ScenarioEvent.Frame;
import frc.robot.sim.scenario.ScenarioJson;
import frc.robot.sim.scenario.Segment;
import frc.robot.sim.scenario.SimAlliance;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.stream.IntStream;
import javafx.application.Platform;
import javafx.beans.property.ReadOnlyObjectWrapper;
import javafx.beans.property.ReadOnlyStringWrapper;
import javafx.fxml.FXML;
import javafx.scene.Node;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ChoiceBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.MenuButton;
import javafx.scene.control.MenuItem;
import javafx.scene.control.Spinner;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import javafx.scene.control.cell.TextFieldTableCell;
import javafx.scene.layout.StackPane;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;
import javafx.util.converter.DefaultStringConverter;
import javafx.util.converter.DoubleStringConverter;

/**
 * Scenario tab: edits one scenario's settings and events, with a field preview where set-pose
 * points and drive-stick arrows can be dragged. Layout: scenario.fxml.
 */
public final class ScenarioController {
  private static final Color POSE_COLOR = Color.web("#43a047");
  private static final Color LINKED_POSE_COLOR = Color.web("#00897b");
  private static final Color EXPECT_COLOR = Color.web("#ba68c8");
  private static final Color ARROW_COLOR = Color.web("#4fc3f7");

  private final AppContext ctx;
  private final Workspace ws;
  private String scenarioId;
  private Scenario scenario;
  private boolean loading;

  @FXML private TextField name;
  @FXML private TextField description;
  @FXML private ComboBox<String> gameData;
  @FXML private CheckBox debugBindings;
  @FXML private CheckBox hasAuto;
  @FXML private ComboBox<String> autoRoutine;
  @FXML private Spinner<Double> autoDuration;
  @FXML private CheckBox hasTeleop;
  @FXML private Spinner<Double> teleopMatchTime;
  @FXML private ToggleButton previewRed;
  @FXML private StackPane fieldHolder;
  @FXML private ChoiceBox<String> segmentChoice;
  @FXML private Label segmentInfo;
  @FXML private TableView<Integer> table;
  @FXML private TableColumn<Integer, String> colIndex;
  @FXML private TableColumn<Integer, String> colTime;
  @FXML private TableColumn<Integer, Double> colWait;
  @FXML private TableColumn<Integer, String> colType;
  @FXML private TableColumn<Integer, String> colDetails;
  @FXML private TableColumn<Integer, String> colLabel;
  @FXML private MenuButton addMenu;
  @FXML private VBox editorHolder;
  @FXML private Label status;
  @FXML private Button saveButton;
  @FXML private Button revertButton;

  private FieldCanvas field;
  private EventEditor editor;

  public ScenarioController(AppContext ctx) {
    this.ctx = ctx;
    this.ws = ctx.ws;
  }

  @FXML
  private void initialize() {
    field = new FieldCanvas(ws::field);
    field.setScene(this::buildScene);
    field.setListener(new FieldEdits());
    fieldHolder.getChildren().add(field);
    editor = new EventEditor(ws::locationNames, n -> n == null ? null : ws.field().locations().get(n));
    editorHolder.getChildren().add(editor);

    gameData.getItems().setAll("", "B", "R");
    autoRoutine.getItems().setAll(Workspace.autoNames());
    for (String type : ScenarioJson.EVENT_TYPES) {
      MenuItem item = new MenuItem(type);
      item.setOnAction(e -> addEvent(type));
      addMenu.getItems().add(item);
    }
    previewRed.selectedProperty().addListener((o, a, b) -> field.redraw());

    // settings → scenario
    Runnable apply = this::applySettings;
    name.textProperty().addListener((o, a, b) -> apply.run());
    description.textProperty().addListener((o, a, b) -> apply.run());
    gameData.valueProperty().addListener((o, a, b) -> apply.run());
    debugBindings.selectedProperty().addListener((o, a, b) -> apply.run());
    hasAuto.selectedProperty().addListener((o, a, b) -> apply.run());
    autoRoutine.valueProperty().addListener((o, a, b) -> apply.run());
    autoDuration.valueProperty().addListener((o, a, b) -> apply.run());
    hasTeleop.selectedProperty().addListener((o, a, b) -> apply.run());
    teleopMatchTime.valueProperty().addListener((o, a, b) -> apply.run());

    setUpTable();
    segmentChoice.valueProperty().addListener((o, a, b) -> {
      if (!loading) {
        reloadTable(-1);
      }
    });
    ws.addListener(() -> Platform.runLater(this::workspaceChanged));
    show(null);
  }

  // ------------------------------------------------------------------ table

  private void setUpTable() {
    colIndex.setCellValueFactory(c -> new ReadOnlyStringWrapper(String.valueOf(c.getValue())));
    colTime.setCellValueFactory(c -> new ReadOnlyStringWrapper(
        validRow(c.getValue()) ? String.format(Locale.ROOT, "%.2f", segment().absoluteTimes()[c.getValue()]) : ""));
    colWait.setCellValueFactory(c -> new ReadOnlyObjectWrapper<>(validRow(c.getValue()) ? events().get(c.getValue()).waitSec() : null));
    colType.setCellValueFactory(c -> new ReadOnlyStringWrapper(validRow(c.getValue()) ? events().get(c.getValue()).type() : ""));
    colDetails.setCellValueFactory(c -> new ReadOnlyStringWrapper(validRow(c.getValue()) ? events().get(c.getValue()).summary() : ""));
    colLabel.setCellValueFactory(c -> {
      String l = validRow(c.getValue()) ? events().get(c.getValue()).label() : null;
      return new ReadOnlyStringWrapper(l == null ? "" : l);
    });

    // inline editing of wait and label
    colWait.setCellFactory(TextFieldTableCell.forTableColumn(new DoubleStringConverter() {
      @Override
      public Double fromString(String s) {
        try {
          return Math.max(0, super.fromString(s));
        } catch (RuntimeException e) {
          return null;
        }
      }
    }));
    colWait.setOnEditCommit(e -> {
      if (e.getNewValue() != null) {
        int i = e.getRowValue();
        replaceEvent(i, events().get(i).withWait(e.getNewValue()), true);
      }
    });
    colLabel.setCellFactory(TextFieldTableCell.forTableColumn(new DefaultStringConverter()));
    colLabel.setOnEditCommit(e -> {
      int i = e.getRowValue();
      String v = e.getNewValue();
      replaceEvent(i, events().get(i).withLabel(v == null || v.isBlank() ? null : v), true);
    });
    colIndex.setEditable(false);
    colTime.setEditable(false);
    colType.setEditable(false);
    colDetails.setEditable(false);

    table.getSelectionModel().selectedItemProperty().addListener((o, a, b) -> showSelected());
  }

  private boolean validRow(Integer i) {
    return i != null && i >= 0 && i < events().size();
  }

  /** Rebuilds the table rows (one per event index) and selects {@code select}. */
  private void reloadTable(int select) {
    table.getItems().setAll(IntStream.range(0, events().size()).boxed().toList());
    selectRow(select);
    updateSegmentInfo();
  }

  private void selectRow(int i) {
    if (i >= 0 && i < table.getItems().size()) {
      table.getSelectionModel().select(i);
      table.scrollTo(Math.max(0, i - 3));
    } else {
      table.getSelectionModel().clearSelection();
    }
    showSelected();
  }

  private int selectedRow() {
    Integer i = table.getSelectionModel().getSelectedItem();
    return i == null ? -1 : i;
  }

  private void showSelected() {
    int i = selectedRow();
    if (!validRow(i)) {
      editor.showNothing();
    } else {
      editor.edit(events().get(i), replacement -> replaceEvent(i, replacement, false));
    }
    field.redraw();
  }

  /** Replaces an event in place (from the form, the table or the field). */
  private void replaceEvent(int index, ScenarioEvent replacement, boolean refreshEditor) {
    if (!validRow(index)) {
      return;
    }
    events().set(index, replacement);
    changed();
    table.refresh(); // later absolute times shift too
    updateSegmentInfo();
    if (refreshEditor) {
      editor.refresh(replacement);
    }
  }

  private void updateSegmentInfo() {
    Segment seg = segment();
    segmentInfo.setText(seg == null ? "" : String.format(Locale.ROOT, "%d events · %.1f s", seg.events().size(), seg.endTime()));
  }

  // ------------------------------------------------------------------ event buttons

  private void addEvent(String type) {
    Segment seg = segment();
    if (seg == null) {
      Ui.info(ctx.stage, "No segment", "Tick Auto or Teleop first.");
      return;
    }
    int at = selectedRow() >= 0 ? selectedRow() + 1 : seg.events().size();
    // keep "end" last
    if (!type.equals("end") && at == seg.events().size() && at > 0 && seg.events().get(at - 1) instanceof ScenarioEvent.End) {
      at--;
    }
    seg.events().add(at, defaultEvent(type));
    afterStructureChange(at);
  }

  @FXML
  private void duplicateEvent() {
    int i = selectedRow();
    if (validRow(i)) {
      events().add(i + 1, events().get(i));
      afterStructureChange(i + 1);
    }
  }

  @FXML
  private void deleteEvent() {
    int i = selectedRow();
    if (validRow(i)) {
      events().remove(i);
      afterStructureChange(Math.min(i, events().size() - 1));
    }
  }

  @FXML
  private void moveUp() {
    move(-1);
  }

  @FXML
  private void moveDown() {
    move(1);
  }

  private void move(int delta) {
    int i = selectedRow();
    int j = i + delta;
    if (!validRow(i) || !validRow(j)) {
      return;
    }
    List<ScenarioEvent> ev = events();
    ScenarioEvent tmp = ev.get(i);
    ev.set(i, ev.get(j));
    ev.set(j, tmp);
    afterStructureChange(j);
  }

  private void afterStructureChange(int select) {
    changed();
    reloadTable(select);
  }

  private ScenarioEvent defaultEvent(String type) {
    List<String> locs = ws.locationNames();
    return switch (type) {
      case "setPose" -> locs.isEmpty()
          ? new ScenarioEvent.SetPose(0.1, null, null, new Pose(2, 2, 0), Frame.ALLIANCE)
          : new ScenarioEvent.SetPose(0.1, null, locs.get(0), null, Frame.ALLIANCE);
      case "drive" -> new ScenarioEvent.Drive(0.5, null, 0.5, 0);
      case "turn" -> new ScenarioEvent.Turn(0.5, null, 0.3);
      case "stopJoysticks" -> new ScenarioEvent.StopJoysticks(1.0, null);
      case "button" -> new ScenarioEvent.Button(0.5, null, ScenarioEvent.GamepadButton.A, ButtonAction.PRESS);
      case "pov" -> new ScenarioEvent.Pov(0.5, null, ScenarioEvent.PovDirection.UP, ButtonAction.PRESS);
      case "trigger" -> new ScenarioEvent.Trigger(0.5, null, ScenarioEvent.TriggerSide.RIGHT, 1.0);
      case "gameData" -> new ScenarioEvent.GameData(0, null, "B");
      case "expect" -> EventEditor.defaultExpect("intake.state", ProbeCatalog.find("intake.state"), 0, null, 1.0);
      case "end" -> new ScenarioEvent.End(1.0, null);
      default -> throw new IllegalArgumentException(type);
    };
  }

  // ------------------------------------------------------------------ settings

  private void applySettings() {
    if (loading || scenario == null) {
      return;
    }
    scenario.setName(name.getText());
    scenario.setDescription(description.getText());
    scenario.setGameData(gameData.getValue());
    scenario.setDebugBindings(debugBindings.isSelected());
    if (hasAuto.isSelected()) {
      if (scenario.getAuto() == null) {
        scenario.setAuto(new Segment(true));
      }
      scenario.getAuto().setAutoRoutine(autoRoutine.getValue());
      scenario.getAuto().setDurationSec(autoDuration.getValue());
    } else if (scenario.getAuto() != null && confirmDropSegment(hasAuto, "auto", scenario.getAuto())) {
      scenario.setAuto(null);
    }
    if (hasTeleop.isSelected()) {
      if (scenario.getTeleop() == null) {
        scenario.setTeleop(new Segment(false));
      }
      double mt = teleopMatchTime.getValue();
      scenario.getTeleop().setStartMatchTime(mt == Segment.DEFAULT_TELEOP_MATCH_TIME ? null : mt);
    } else if (scenario.getTeleop() != null && confirmDropSegment(hasTeleop, "teleop", scenario.getTeleop())) {
      scenario.setTeleop(null);
    }
    updateSettingEnables();
    changed();
    refreshSegments();
  }

  private boolean confirmDropSegment(CheckBox box, String which, Segment seg) {
    if (seg.events().isEmpty() || Ui.confirm(ctx.stage, "Remove segment",
        "Remove the " + which + " segment and its " + seg.events().size() + " events?")) {
      return true;
    }
    loading = true;
    box.setSelected(true);
    loading = false;
    return false;
  }

  private void updateSettingEnables() {
    autoRoutine.setDisable(!hasAuto.isSelected());
    autoDuration.setDisable(!hasAuto.isSelected());
    teleopMatchTime.setDisable(!hasTeleop.isSelected());
  }

  // ------------------------------------------------------------------ showing a scenario

  /** Shows a scenario (or nothing when id is null). */
  void show(String id) {
    scenarioId = id;
    scenario = id == null ? null : ws.scenario(id);
    loading = true;
    boolean on = scenario != null;
    for (Node n : new Node[] {name, description, gameData, debugBindings, hasAuto, autoRoutine, autoDuration,
        hasTeleop, teleopMatchTime, segmentChoice, table, addMenu, saveButton, revertButton, previewRed}) {
      n.setDisable(!on);
    }
    if (on) {
      name.setText(scenario.getName());
      description.setText(scenario.getDescription());
      gameData.setValue(scenario.getGameData());
      debugBindings.setSelected(scenario.isDebugBindings());
      hasAuto.setSelected(scenario.getAuto() != null);
      if (scenario.getAuto() != null) {
        autoRoutine.setValue(scenario.getAuto().getAutoRoutine());
        Double d = scenario.getAuto().getDurationSec();
        autoDuration.getValueFactory().setValue(d == null ? Segment.DEFAULT_AUTO_DURATION : d);
      }
      hasTeleop.setSelected(scenario.getTeleop() != null);
      if (scenario.getTeleop() != null) {
        Double mt = scenario.getTeleop().getStartMatchTime();
        teleopMatchTime.getValueFactory().setValue(mt == null ? Segment.DEFAULT_TELEOP_MATCH_TIME : mt);
      }
      updateSettingEnables();
    } else {
      name.setText("");
      description.setText("");
    }
    loading = false;
    refreshSegments();
    refreshStatus();
  }

  String currentId() {
    return scenarioId;
  }

  /** Selects an event (used when jumping from a run result). */
  void selectEvent(String segmentName, int index) {
    if (scenario != null && segmentChoice.getItems().contains(segmentName)) {
      segmentChoice.setValue(segmentName);
      selectRow(index);
    }
  }

  private void refreshSegments() {
    boolean was = loading;
    loading = true;
    String prev = segmentChoice.getValue();
    List<String> items = new ArrayList<>();
    if (scenario != null && scenario.getAuto() != null) {
      items.add("auto");
    }
    if (scenario != null && scenario.getTeleop() != null) {
      items.add("teleop");
    }
    segmentChoice.getItems().setAll(items);
    segmentChoice.setValue(items.contains(prev) ? prev : items.isEmpty() ? null : items.get(items.size() - 1));
    loading = was;
    int keep = selectedRow();
    reloadTable(keep);
  }

  private Segment segment() {
    if (scenario == null) {
      return null;
    }
    return "auto".equals(segmentChoice.getValue()) ? scenario.getAuto() : scenario.getTeleop();
  }

  private List<ScenarioEvent> events() {
    Segment s = segment();
    return s == null ? List.of() : s.events();
  }

  private void changed() {
    if (scenarioId != null) {
      ws.markDirty(scenarioId);
    }
    field.redraw();
  }

  /** Reloads the view if the shown scenario was replaced (revert / reload from disk). */
  private void workspaceChanged() {
    if (scenarioId != null && ws.scenario(scenarioId) != scenario) {
      int keep = selectedRow();
      show(ws.scenario(scenarioId) == null ? null : scenarioId);
      selectRow(keep);
    } else {
      refreshStatus();
    }
  }

  private void refreshStatus() {
    status.getStyleClass().removeAll("status-ok", "status-error", "hint");
    if (scenario == null) {
      status.setText("Select or create a scenario on the left.");
      status.getStyleClass().add("hint");
      status.setTooltip(null);
      return;
    }
    List<String> problems = ws.validate(scenario);
    String dirty = ws.isDirty(scenarioId) ? " (unsaved)" : "";
    if (problems.isEmpty()) {
      status.setText("✔ " + scenarioId + ".json" + dirty + " is valid");
      status.getStyleClass().add("status-ok");
      status.setTooltip(null);
    } else {
      status.setText("✖ " + scenarioId + ".json" + dirty + ": " + problems.size() + " problem(s) · " + problems.get(0));
      status.getStyleClass().add("status-error");
      status.setTooltip(new javafx.scene.control.Tooltip(String.join("\n", problems)));
    }
  }

  @FXML
  private void save() {
    if (scenarioId == null) {
      return;
    }
    try {
      List<String> problems = ws.saveScenario(scenarioId);
      if (!problems.isEmpty()) {
        Ui.warn(ctx.stage, "Invalid scenario", "Not saved. Fix these first:\n\n" + String.join("\n", problems));
      }
    } catch (IOException e) {
      Ui.error(ctx.stage, "Save failed", e.getMessage());
    }
  }

  @FXML
  private void revert() {
    if (scenarioId == null) {
      return;
    }
    try {
      ws.revertScenario(scenarioId);
      show(ws.scenario(scenarioId) == null ? null : scenarioId);
    } catch (IOException e) {
      Ui.error(ctx.stage, "Revert failed", e.getMessage());
    }
  }

  // ------------------------------------------------------------------ field preview

  private SimAlliance previewAlliance() {
    return previewRed.isSelected() ? SimAlliance.RED : SimAlliance.BLUE;
  }

  /** Driver-relative stick → field direction for the preview alliance (its own inverse). */
  private double[] stickToField(double x, double y) {
    return previewAlliance() == SimAlliance.RED ? new double[] {-x, -y} : new double[] {x, y};
  }

  /** Field pose shown for a stored (BLUE-frame) pose; also maps a dragged pose back (flip is self-inverse). */
  private Pose previewFlip(Pose pose, Frame frame) {
    FieldLocations f = ws.field();
    return previewAlliance() == SimAlliance.RED && frame != Frame.ABSOLUTE
        ? pose.flipped(f.getFieldLength(), f.getFieldWidth())
        : pose;
  }

  private FieldCanvas.Scene buildScene() {
    Segment seg = segment();
    if (seg == null) {
      return FieldCanvas.Scene.empty();
    }
    FieldLocations f = ws.field();
    SimAlliance alliance = previewAlliance();
    int selected = selectedRow();
    List<FieldCanvas.Marker> markers = new ArrayList<>();
    List<FieldCanvas.Arrow> arrows = new ArrayList<>();
    List<FieldCanvas.PathLine> path = new ArrayList<>();

    // The simulator puts the robot at the origin before each test; teleop after an auto starts
    // wherever the auto ended, which the editor can't know.
    double px = 0;
    double py = 0;
    double heading = 0;
    double sx = 0;
    double sy = 0;
    double turn = 0;
    double t = 0;
    double[] times = seg.absoluteTimes();
    List<ScenarioEvent> ev = seg.events();
    for (int i = 0; i <= ev.size(); i++) {
      double until = i < ev.size() ? times[i] : seg.endTime();
      double dt = until - t;
      if (dt > 0 && (sx != 0 || sy != 0)) {
        double[] d = stickToField(sx, sy);
        // stop at the field walls; everything else (bumps, robots) is ignored
        double nx = Ui.clamp(px + d[0] * f.getPreviewMaxSpeedMps() * dt, 0, f.getFieldLength());
        double ny = Ui.clamp(py + d[1] * f.getPreviewMaxSpeedMps() * dt, 0, f.getFieldWidth());
        path.add(new FieldCanvas.PathLine(px, py, nx, ny));
        px = nx;
        py = ny;
      }
      heading += turn * f.getPreviewMaxTurnDegPerSec() * Math.max(0, dt);
      t = until;
      if (i == ev.size()) {
        break;
      }
      ScenarioEvent e = ev.get(i);
      boolean sel = i == selected;
      String tag = "#" + i + (e.label() != null ? " " + e.label() : "");
      switch (e) {
        case ScenarioEvent.SetPose sp -> {
          Pose p = f.resolve(sp.location(), sp.pose(), sp.frame(), alliance);
          if (p != null) {
            px = p.x();
            py = p.y();
            heading = p.deg();
            markers.add(new FieldCanvas.Marker(i, p, tag, sp.location() != null ? LINKED_POSE_COLOR : POSE_COLOR, sel,
                sp.location() == null, false));
          }
        }
        case ScenarioEvent.Drive d -> {
          sx = d.x();
          sy = d.y();
          double[] v = stickToField(d.x(), d.y());
          arrows.add(new FieldCanvas.Arrow(i, px, py, px + v[0] * FieldCanvas.STICK_ARROW_SCALE,
              py + v[1] * FieldCanvas.STICK_ARROW_SCALE, ARROW_COLOR, sel, true));
        }
        case ScenarioEvent.Turn tr -> turn = tr.rate();
        case ScenarioEvent.StopJoysticks s -> {
          sx = 0;
          sy = 0;
          turn = 0;
        }
        case ScenarioEvent.Expect x when x.isPoseCheck() -> {
          Pose p = f.resolve(x.location(), x.pose(), x.frame(), alliance);
          if (p != null) {
            markers.add(new FieldCanvas.Marker(i, p, tag + " (expect)", EXPECT_COLOR, sel, x.location() == null, true));
          }
        }
        default -> {}
      }
    }
    markers.add(0, new FieldCanvas.Marker("end", new Pose(px, py, Pose.normalizeDeg(heading)), "end (estimate)",
        Color.web("#bdbdbd"), false, false, true));
    String caption = String.format(Locale.ROOT,
        "Path estimate: %.1f m/s at full stick, stops at walls, ignores bumps and collisions%s. Each test starts at the origin.",
        f.getPreviewMaxSpeedMps(), seg.isAuto() ? "; auto paths are not previewed" : "");
    if (!seg.isAuto() && scenario.getAuto() != null) {
      caption += " Teleop really starts where the auto ended.";
    }
    return new FieldCanvas.Scene(markers, arrows, path, caption);
  }

  /** Applies drags on the field preview back to the scenario. */
  private final class FieldEdits implements FieldCanvas.Listener {
    @Override
    public void selected(Object key) {
      if (key instanceof Integer i) {
        selectRow(i);
      }
    }

    @Override
    public void markerMoved(Object key, double x, double y, boolean finished) {
      if (!(key instanceof Integer i) || !validRow(i)) {
        return;
      }
      switch (events().get(i)) {
        case ScenarioEvent.SetPose sp when sp.pose() != null -> {
          Pose stored = previewFlip(new Pose(x, y, previewFlip(sp.pose(), sp.frame()).deg()), sp.frame());
          replaceEvent(i, new ScenarioEvent.SetPose(sp.waitSec(), sp.label(), null, stored, sp.frame()), true);
        }
        case ScenarioEvent.Expect xp when xp.pose() != null -> {
          Pose stored = previewFlip(new Pose(x, y, previewFlip(xp.pose(), xp.frame()).deg()), xp.frame());
          replaceEvent(i, withExpectPose(xp, stored), true);
        }
        default -> {}
      }
    }

    @Override
    public void markerRotated(Object key, double deg, boolean finished) {
      if (!(key instanceof Integer i) || !validRow(i)) {
        return;
      }
      switch (events().get(i)) {
        case ScenarioEvent.SetPose sp when sp.pose() != null -> {
          Pose stored = previewFlip(previewFlip(sp.pose(), sp.frame()).withDeg(deg), sp.frame());
          replaceEvent(i, new ScenarioEvent.SetPose(sp.waitSec(), sp.label(), null, stored, sp.frame()), true);
        }
        case ScenarioEvent.Expect xp when xp.pose() != null ->
            replaceEvent(i, withExpectPose(xp, previewFlip(previewFlip(xp.pose(), xp.frame()).withDeg(deg), xp.frame())), true);
        default -> {}
      }
    }

    @Override
    public void arrowTipMoved(Object key, double x, double y, boolean finished) {
      if (!(key instanceof Integer i) || !validRow(i) || !(events().get(i) instanceof ScenarioEvent.Drive d)) {
        return;
      }
      for (FieldCanvas.Arrow a : buildScene().arrows()) {
        if (a.key().equals(i)) { // the arrow starts at the estimated robot position
          double[] stick = stickToField((x - a.x0()) / FieldCanvas.STICK_ARROW_SCALE, (y - a.y0()) / FieldCanvas.STICK_ARROW_SCALE);
          double[] v = EventEditor.clampStick(stick[0], stick[1]);
          replaceEvent(i, new ScenarioEvent.Drive(d.waitSec(), d.label(), v[0], v[1]), true);
          return;
        }
      }
    }
  }

  private static ScenarioEvent.Expect withExpectPose(ScenarioEvent.Expect x, Pose p) {
    return new ScenarioEvent.Expect(x.waitSec(), x.label(), x.probe(), x.equalsValue(), x.min(), x.max(), null, p,
        x.frame(), x.tolerance(), x.headingTolDeg(), x.within());
  }
}
