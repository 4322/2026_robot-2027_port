package frc.robot.simgui;

import frc.robot.sim.scenario.Scenario;
import frc.robot.sim.scenario.ScenarioEvent;
import frc.robot.sim.scenario.Segment;
import java.io.IOException;
import java.util.List;
import java.util.Optional;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.ListCell;
import javafx.scene.control.ListView;
import javafx.scene.control.TabPane;
import javafx.scene.control.Tooltip;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyCodeCombination;
import javafx.scene.input.KeyCombination;
import javafx.stage.WindowEvent;

/** Main window: scenario list, tabs, save/reload, and the close-with-unsaved-changes check. */
public final class MainController {
  private final AppContext ctx;
  private final Workspace ws;

  @FXML private Label dirLabel;
  @FXML private Label statusLabel;
  @FXML private ListView<String> scenarioList;
  @FXML private TabPane tabs;
  @FXML private ScenarioController scenarioTabController;

  private boolean refreshingList;

  public MainController(AppContext ctx) {
    this.ctx = ctx;
    this.ws = ctx.ws;
  }

  @FXML
  private void initialize() {
    dirLabel.setText(ws.paths.dir().toString());
    scenarioList.setCellFactory(list -> new ScenarioCell());
    scenarioList.getSelectionModel().selectedItemProperty().addListener((obs, old, id) -> {
      if (!refreshingList) {
        scenarioTabController.show(id);
        tabs.getSelectionModel().select(0);
      }
    });
    ctx.jumpTo = this::jumpTo;
    ws.addListener(() -> Platform.runLater(this::refresh));
    refresh();

    Platform.runLater(() -> {
      ctx.stage.setOnCloseRequest(this::closeRequested);
      ctx.stage.getScene().getAccelerators().put(
          new KeyCodeCombination(KeyCode.S, KeyCombination.SHORTCUT_DOWN), this::saveAll);
      if (!ws.loadErrors.isEmpty()) {
        Ui.warn(ctx.stage, "Load errors",
            "Some files couldn't be loaded and were skipped:\n\n" + String.join("\n", ws.loadErrors));
      }
    });
  }

  private void refresh() {
    refreshingList = true;
    String sel = scenarioList.getSelectionModel().getSelectedItem();
    scenarioList.getItems().setAll(ws.scenarioIds());
    if (sel != null && ws.scenario(sel) != null) {
      scenarioList.getSelectionModel().select(sel);
    }
    refreshingList = false;
    if (sel != null && ws.scenario(sel) == null && sel.equals(scenarioTabController.currentId())) {
      scenarioTabController.show(null);
    }
    scenarioList.refresh();
    boolean dirty = ws.hasUnsavedChanges();
    statusLabel.setText(dirty ? "Unsaved changes (Ctrl+S saves all)" : "All changes saved");
    statusLabel.getStyleClass().setAll("label", dirty ? "status-warn" : "status-ok");
    ctx.stage.setTitle("Simulator Scenario Editor" + (dirty ? " *" : ""));
  }

  /** List cell showing dirty (*) and invalid (⚠) state, with the scenario name as tooltip. */
  private final class ScenarioCell extends ListCell<String> {
    @Override
    protected void updateItem(String id, boolean empty) {
      super.updateItem(id, empty);
      getStyleClass().removeAll("dirty", "invalid");
      if (empty || id == null) {
        setText(null);
        setTooltip(null);
        return;
      }
      Scenario s = ws.scenario(id);
      boolean dirty = ws.isDirty(id);
      boolean invalid = s != null && !ws.validate(s).isEmpty();
      setText((dirty ? "● " : "") + id + (invalid ? "  ⚠" : ""));
      if (dirty) {
        getStyleClass().add("dirty");
      }
      if (invalid) {
        getStyleClass().add("invalid");
      }
      setTooltip(s == null ? null : new Tooltip(s.getName() + (s.getDescription().isEmpty() ? "" : "\n" + s.getDescription())));
    }
  }

  // ------------------------------------------------------------------ scenario list actions

  @FXML
  private void newScenario() {
    create(null);
  }

  @FXML
  private void duplicateScenario() {
    String id = scenarioList.getSelectionModel().getSelectedItem();
    if (id != null) {
      create(id);
    }
  }

  private void create(String copyOfId) {
    Optional<String> answer = Ui.ask(ctx.stage, "New scenario", "File id (lowercase, digits, - or _):",
        copyOfId == null ? "new-scenario" : copyOfId + "-copy");
    if (answer.isEmpty()) {
      return;
    }
    String id = answer.get();
    if (!Workspace.isValidId(id)) {
      Ui.warn(ctx.stage, "Invalid id", "Use lowercase letters, digits, - and _ (e.g. \"shoot-from-trench\").");
      return;
    }
    if (ws.scenario(id) != null) {
      Ui.warn(ctx.stage, "Already exists", "\"" + id + "\" already exists.");
      return;
    }
    Scenario s;
    if (copyOfId != null) {
      Scenario orig = ws.scenario(copyOfId);
      s = orig.copy();
      s.setName(orig.getName() + " (copy)");
    } else {
      s = new Scenario(id);
      Segment teleop = new Segment(false);
      teleop.events().add(new ScenarioEvent.End(1.0, null));
      s.setTeleop(teleop);
    }
    ws.addScenario(id, s);
    Platform.runLater(() -> {
      scenarioList.getSelectionModel().select(id);
      scenarioList.scrollTo(id);
    });
  }

  @FXML
  private void deleteScenario() {
    String id = scenarioList.getSelectionModel().getSelectedItem();
    if (id == null) {
      return;
    }
    List<String> suites = ws.suiteIds().stream()
        .filter(s -> ws.suite(s).runs().stream().anyMatch(r -> id.equals(r.scenario()))).toList();
    String warn = suites.isEmpty() ? "" : "\n\nIt is used by suite(s): " + String.join(", ", suites);
    if (Ui.confirm(ctx.stage, "Delete scenario", "Delete " + id + ".json?" + warn)) {
      try {
        ws.deleteScenario(id);
        scenarioTabController.show(null);
      } catch (IOException e) {
        Ui.error(ctx.stage, "Delete failed", e.getMessage());
      }
    }
  }

  /** From a run result: open the scenario and select the event ("segment:index"). */
  private void jumpTo(String scenarioId, String where) {
    if (ws.scenario(scenarioId) == null) {
      return;
    }
    scenarioList.getSelectionModel().select(scenarioId);
    scenarioList.scrollTo(scenarioId);
    scenarioTabController.show(scenarioId);
    tabs.getSelectionModel().select(0);
    String[] parts = where.split(":");
    scenarioTabController.selectEvent(parts[0], Integer.parseInt(parts[1]));
  }

  // ------------------------------------------------------------------ save / reload / close

  @FXML
  private void saveAll() {
    trySaveAll();
  }

  private boolean trySaveAll() {
    List<String> problems = ws.saveAll();
    if (!problems.isEmpty()) {
      Ui.warn(ctx.stage, "Save all", "Not everything was saved:\n\n" + String.join("\n", problems));
      return false;
    }
    return true;
  }

  @FXML
  private void reload() {
    if (!ws.hasUnsavedChanges() || Ui.confirm(ctx.stage, "Reload", "Discard unsaved changes and reload from disk?")) {
      ws.reload();
      scenarioTabController.show(scenarioList.getSelectionModel().getSelectedItem());
    }
  }

  private void closeRequested(WindowEvent e) {
    if (ws.hasUnsavedChanges()) {
      Optional<Boolean> save = Ui.yesNoCancel(ctx.stage, "Unsaved changes", "Save changes before closing?");
      if (save.isEmpty() || (save.get() && !trySaveAll())) {
        e.consume();
        return;
      }
    }
    if (ctx.process.isRunning()) {
      if (!Ui.confirm(ctx.stage, "Simulation running", "A simulation is running. Stop it and close?")) {
        e.consume();
        return;
      }
      ctx.process.kill();
    }
    Platform.exit();
  }
}
