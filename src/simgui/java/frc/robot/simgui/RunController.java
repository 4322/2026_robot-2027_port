package frc.robot.simgui;

import frc.robot.sim.scenario.RunConfig;
import frc.robot.sim.scenario.RunReport;
import frc.robot.sim.scenario.Scenario;
import frc.robot.sim.scenario.ScenarioJson;
import frc.robot.sim.scenario.SimAlliance;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.attribute.FileTime;
import java.util.ArrayList;
import java.util.List;
import java.util.Locale;
import java.util.function.Function;
import javafx.application.Platform;
import javafx.beans.property.ReadOnlyStringWrapper;
import javafx.fxml.FXML;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.ListView;
import javafx.scene.control.RadioButton;
import javafx.scene.control.SelectionMode;
import javafx.scene.control.TableCell;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableRow;
import javafx.scene.control.TableView;
import javafx.scene.control.TextArea;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;

/**
 * Run tab: launches the simulation (scripted suite / scenarios, or manual gamepad), shows the
 * console and the pass/fail report. Layout: run.fxml.
 */
public final class RunController {
  private static final int MAX_CONSOLE_CHARS = 400_000;

  /** One row of the results table. {@code scenarioId}/{@code where} locate a check, or are null. */
  record ResultRow(String result, String test, String alliance, String check, String time, String observed,
      String scenarioId, String where) {}

  private final AppContext ctx;
  private final Workspace ws;
  private FileTime reportTimeBeforeRun;

  @FXML private RadioButton scripted;
  @FXML private RadioButton manual;
  @FXML private GridPane scriptedPane;
  @FXML private HBox manualPane;
  @FXML private RadioButton bySuite;
  @FXML private RadioButton byScenarios;
  @FXML private ComboBox<String> suite;
  @FXML private ListView<String> scenarioList;
  @FXML private CheckBox blue;
  @FXML private CheckBox red;
  @FXML private CheckBox headless;
  @FXML private ComboBox<SimAlliance> manualAlliance;
  @FXML private ComboBox<RunConfig.ManualRobotMode> manualMode;
  @FXML private ComboBox<String> manualStart;
  @FXML private ComboBox<String> manualAuto;
  @FXML private Button runButton;
  @FXML private Button stopButton;
  @FXML private CheckBox offline;
  @FXML private Label state;
  @FXML private TextArea console;
  @FXML private TableView<ResultRow> results;
  @FXML private TableColumn<ResultRow, String> colResult;
  @FXML private TableColumn<ResultRow, String> colTest;
  @FXML private TableColumn<ResultRow, String> colAlliance;
  @FXML private TableColumn<ResultRow, String> colCheck;
  @FXML private TableColumn<ResultRow, String> colTime;
  @FXML private TableColumn<ResultRow, String> colObserved;

  public RunController(AppContext ctx) {
    this.ctx = ctx;
    this.ws = ctx.ws;
  }

  @FXML
  private void initialize() {
    scriptedPane.visibleProperty().bind(scripted.selectedProperty());
    manualPane.visibleProperty().bind(manual.selectedProperty());
    suite.disableProperty().bind(bySuite.selectedProperty().not());
    scenarioList.disableProperty().bind(byScenarios.selectedProperty().not());
    blue.disableProperty().bind(byScenarios.selectedProperty().not());
    red.disableProperty().bind(byScenarios.selectedProperty().not());
    scenarioList.getSelectionModel().setSelectionMode(SelectionMode.MULTIPLE);
    manualAlliance.getItems().setAll(SimAlliance.values());
    manualAlliance.setValue(SimAlliance.BLUE);
    manualMode.getItems().setAll(RunConfig.ManualRobotMode.values());
    manualMode.setValue(RunConfig.ManualRobotMode.TELEOPERATED);

    column(colResult, ResultRow::result);
    column(colTest, ResultRow::test);
    column(colAlliance, ResultRow::alliance);
    column(colCheck, ResultRow::check);
    column(colTime, ResultRow::time);
    column(colObserved, ResultRow::observed);
    colResult.setCellFactory(col -> new TableCell<>() {
      @Override
      protected void updateItem(String v, boolean empty) {
        super.updateItem(v, empty);
        setText(empty ? null : v);
        getStyleClass().removeAll("result-pass", "result-fail", "result-other");
        if (!empty && v != null) {
          getStyleClass().add(switch (v) {
            case "PASS", "pass" -> "result-pass";
            case "FAIL", "fail", "ERROR" -> "result-fail";
            default -> "result-other";
          });
        }
      }
    });
    results.setRowFactory(tv -> {
      TableRow<ResultRow> row = new TableRow<>();
      row.setOnMouseClicked(e -> {
        if (e.getClickCount() == 2 && row.getItem() != null && row.getItem().scenarioId() != null) {
          ctx.jumpTo.accept(row.getItem().scenarioId(), row.getItem().where());
        }
      });
      return row;
    });

    ws.addListener(() -> Platform.runLater(this::refreshChoices));
    refreshChoices();
    updateButtons();
    loadReport(false);
  }

  private static void column(TableColumn<ResultRow, String> col, Function<ResultRow, String> getter) {
    col.setCellValueFactory(c -> new ReadOnlyStringWrapper(getter.apply(c.getValue())));
  }

  private void refreshChoices() {
    String selSuite = suite.getValue();
    suite.getItems().setAll(ws.suiteIds());
    suite.setValue(selSuite != null ? selSuite : ws.suiteIds().contains("default") ? "default" : null);

    List<String> selected = new ArrayList<>(scenarioList.getSelectionModel().getSelectedItems());
    scenarioList.getItems().setAll(ws.scenarioIds());
    for (String s : selected) {
      scenarioList.getSelectionModel().select(s);
    }

    String start = manualStart.getValue();
    manualStart.getItems().setAll("");
    manualStart.getItems().addAll(ws.locationNames());
    manualStart.setValue(start == null ? "" : start);

    String auto = manualAuto.getValue();
    manualAuto.getItems().setAll("");
    manualAuto.getItems().addAll(Workspace.autoNames());
    manualAuto.setValue(auto == null ? "" : auto);
  }

  private void updateButtons() {
    boolean running = ctx.process.isRunning();
    runButton.setDisable(running);
    stopButton.setDisable(!running);
    state.setText(running ? "● Running" : "Idle");
    state.getStyleClass().removeAll("status-warn", "hint");
    state.getStyleClass().add(running ? "status-warn" : "hint");
  }

  // ------------------------------------------------------------------ running

  @FXML
  private void run() {
    RunConfig config = buildConfig();
    if (config == null) {
      return;
    }
    // save edits so the sim runs what's on screen
    if (ws.hasUnsavedChanges()) {
      List<String> problems = ws.saveAll();
      if (!problems.isEmpty() && !Ui.confirm(ctx.stage, "Unsaved changes",
          "Some files couldn't be saved (the sim will use the versions on disk):\n\n" + String.join("\n", problems)
              + "\n\nRun anyway?")) {
        return;
      }
    }
    List<String> invalid = new ArrayList<>();
    for (String id : scenariosInRun(config)) {
      Scenario s = ws.scenario(id);
      if (s == null) {
        invalid.add(id + ": missing");
      } else {
        ws.validate(s).forEach(p -> invalid.add(id + ": " + p));
      }
    }
    if (!invalid.isEmpty()) {
      Ui.warn(ctx.stage, "Invalid scenarios", "Fix these first (the simulator would skip them):\n\n" + String.join("\n", invalid));
      return;
    }
    try {
      ScenarioJson.writeRunConfig(ws.paths.runConfig(), config);
      reportTimeBeforeRun = Files.exists(ws.paths.latestResults()) ? Files.getLastModifiedTime(ws.paths.latestResults()) : null;
      results.getItems().clear();
      boolean noWindow = config.getMode() == RunConfig.Mode.SCRIPTED && headless.isSelected();
      ctx.process.start(noWindow, offline.isSelected(), this::appendLine,
          code -> Platform.runLater(() -> finished(code)));
    } catch (IOException e) {
      Ui.error(ctx.stage, "Could not start", e.getMessage());
    }
    updateButtons();
  }

  @FXML
  private void stop() {
    ctx.process.stop(this::appendLine);
  }

  @FXML
  private void clearConsole() {
    console.clear();
  }

  private RunConfig buildConfig() {
    RunConfig c = new RunConfig();
    if (manual.isSelected()) {
      c.setMode(RunConfig.Mode.MANUAL);
      c.setManualAlliance(manualAlliance.getValue());
      c.setManualRobotMode(manualMode.getValue());
      c.setStartLocation(blankToNull(manualStart.getValue()));
      c.setAutoRoutine(blankToNull(manualAuto.getValue()));
      return c;
    }
    if (bySuite.isSelected()) {
      if (suite.getValue() == null) {
        Ui.info(ctx.stage, "Nothing to run", "Pick a suite.");
        return null;
      }
      c.setSuite(suite.getValue());
      return c;
    }
    List<String> picked = scenarioList.getSelectionModel().getSelectedItems();
    if (picked.isEmpty() || (!blue.isSelected() && !red.isSelected())) {
      Ui.info(ctx.stage, "Nothing to run", "Pick at least one scenario and one alliance.");
      return null;
    }
    c.scenarios().addAll(picked);
    if (blue.isSelected()) {
      c.alliances().add(SimAlliance.BLUE);
    }
    if (red.isSelected()) {
      c.alliances().add(SimAlliance.RED);
    }
    return c;
  }

  private static String blankToNull(String s) {
    return s == null || s.isBlank() ? null : s;
  }

  private List<String> scenariosInRun(RunConfig c) {
    if (c.getMode() == RunConfig.Mode.MANUAL) {
      return List.of();
    }
    if (c.getSuite() == null) {
      return c.scenarios();
    }
    var s = ws.suite(c.getSuite());
    return s == null ? List.of() : s.runs().stream().filter(r -> !r.allAutos()).map(r -> r.scenario()).distinct().toList();
  }

  private void appendLine(String line) {
    Platform.runLater(() -> {
      console.appendText(line + "\n");
      int len = console.getLength();
      if (len > MAX_CONSOLE_CHARS) {
        console.deleteText(0, len - MAX_CONSOLE_CHARS);
      }
    });
  }

  private void finished(int code) {
    appendLine("[editor] Simulation exited with code " + code
        + (code == 0 ? " (all passed)" : " (failures, or the build/sim failed; see above)"));
    updateButtons();
    loadReport(true);
  }

  /** Shows results/latest.json. When {@code mustBeNew}, ignores a report left from an older run. */
  private void loadReport(boolean mustBeNew) {
    try {
      if (!Files.exists(ws.paths.latestResults())) {
        return;
      }
      FileTime t = Files.getLastModifiedTime(ws.paths.latestResults());
      if (mustBeNew && reportTimeBeforeRun != null && !t.toInstant().isAfter(reportTimeBeforeRun.toInstant())) {
        appendLine("[editor] No new results report was written.");
        return;
      }
      RunReport report = ScenarioJson.readRunReport(ws.paths.latestResults());
      List<ResultRow> rows = new ArrayList<>();
      for (String e : report.errors()) {
        rows.add(new ResultRow("ERROR", "(run)", "", e, "", "", null, null));
      }
      for (RunReport.Test test : report.tests()) {
        rows.add(new ResultRow(test.passed() ? "PASS" : "FAIL", test.name(), test.alliance().name(),
            test.checks().size() + " check(s)", "", "", null, null));
        for (String e : test.errors()) {
          rows.add(new ResultRow("ERROR", "", "", e, "", "", null, null));
        }
        for (RunReport.Check c : test.checks()) {
          rows.add(new ResultRow(c.passed() ? "pass" : "fail", "", "", c.label(),
              String.format(Locale.ROOT, "%.2f", c.simTime()), c.actual(),
              test.scenarioId(), c.segment() + ":" + c.eventIndex()));
        }
      }
      if (report.isStoppedEarly()) {
        rows.add(new ResultRow("STOP", "(stopped early)", "", "", "", "", null, null));
      }
      results.getItems().setAll(rows);
    } catch (IOException e) {
      appendLine("[editor] Could not read results: " + e.getMessage());
    }
  }
}
