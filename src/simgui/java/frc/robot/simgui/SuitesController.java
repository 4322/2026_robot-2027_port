package frc.robot.simgui;

import frc.robot.sim.scenario.SimAlliance;
import frc.robot.sim.scenario.Suite;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.stream.IntStream;
import javafx.application.Platform;
import javafx.beans.property.ReadOnlyObjectWrapper;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleStringProperty;
import javafx.collections.FXCollections;
import javafx.collections.ObservableList;
import javafx.fxml.FXML;
import javafx.scene.control.ListCell;
import javafx.scene.control.ListView;
import javafx.scene.control.TableCell;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.TextField;
import javafx.scene.control.cell.CheckBoxTableCell;
import javafx.scene.control.cell.ComboBoxTableCell;
import javafx.scene.control.cell.TextFieldTableCell;
import javafx.util.converter.DoubleStringConverter;

/** Suites tab: edits suite files (ordered scenario × alliance runs). Layout: suites.fxml. */
public final class SuitesController {
  static final String ALL_AUTOS = "(all autos)";

  private final AppContext ctx;
  private final Workspace ws;
  private final ObservableList<String> scenarioChoices = FXCollections.observableArrayList();
  private String current;
  private boolean loading;

  @FXML private ListView<String> suiteList;
  @FXML private TextField name;
  @FXML private TableView<Integer> table;
  @FXML private TableColumn<Integer, String> colScenario;
  @FXML private TableColumn<Integer, Boolean> colBlue;
  @FXML private TableColumn<Integer, Boolean> colRed;
  @FXML private TableColumn<Integer, Double> colDuration;

  public SuitesController(AppContext ctx) {
    this.ctx = ctx;
    this.ws = ctx.ws;
  }

  @FXML
  private void initialize() {
    suiteList.setCellFactory(l -> new ListCell<>() {
      @Override
      protected void updateItem(String id, boolean empty) {
        super.updateItem(id, empty);
        setText(empty || id == null ? null : (ws.isSuiteDirty(id) ? "● " : "") + id);
      }
    });
    suiteList.getSelectionModel().selectedItemProperty().addListener((o, a, id) -> {
      if (!loading) {
        current = id;
        showCurrent();
      }
    });
    name.textProperty().addListener((o, a, text) -> {
      if (!loading && suite() != null && !text.equals(suite().getName())) {
        suite().setName(text);
        ws.markSuiteDirty(current);
      }
    });

    colScenario.setCellValueFactory(c -> {
      Suite.Run r = run(c.getValue());
      return new SimpleStringProperty(r == null ? "" : r.allAutos() ? ALL_AUTOS : r.scenario());
    });
    colScenario.setCellFactory(ComboBoxTableCell.forTableColumn(scenarioChoices));
    colScenario.setOnEditCommit(e -> {
      Suite.Run r = run(e.getRowValue());
      set(e.getRowValue(), ALL_AUTOS.equals(e.getNewValue())
          ? new Suite.Run(null, true, r.alliances(), r.autoDuration())
          : new Suite.Run(e.getNewValue(), false, r.alliances(), null));
    });
    colBlue.setCellValueFactory(c -> allianceProperty(c.getValue(), SimAlliance.BLUE));
    colBlue.setCellFactory(CheckBoxTableCell.forTableColumn(colBlue));
    colRed.setCellValueFactory(c -> allianceProperty(c.getValue(), SimAlliance.RED));
    colRed.setCellFactory(CheckBoxTableCell.forTableColumn(colRed));
    colDuration.setCellValueFactory(c -> {
      Suite.Run r = run(c.getValue());
      return new ReadOnlyObjectWrapper<>(r == null || !r.allAutos() ? null : r.autoDuration() == null ? 20.0 : r.autoDuration());
    });
    colDuration.setCellFactory(col -> {
      TableCell<Integer, Double> cell = new TextFieldTableCell<>(new DoubleStringConverter() {
        @Override
        public Double fromString(String s) {
          try {
            return Math.max(0, super.fromString(s));
          } catch (RuntimeException e) {
            return null;
          }
        }
      }) {
        @Override
        public void startEdit() {
          Suite.Run r = run(getTableRow() == null ? null : getTableRow().getItem());
          if (r != null && r.allAutos()) { // only all-autos rows have a duration
            super.startEdit();
          }
        }
      };
      return cell;
    });
    colDuration.setOnEditCommit(e -> {
      Suite.Run r = run(e.getRowValue());
      if (e.getNewValue() != null) {
        set(e.getRowValue(), new Suite.Run(r.scenario(), r.allAutos(), r.alliances(), e.getNewValue()));
      }
    });

    ws.addListener(() -> Platform.runLater(this::refresh));
    refresh();
  }

  /** A checkbox value bound to whether a run includes an alliance. */
  private SimpleBooleanProperty allianceProperty(Integer row, SimAlliance alliance) {
    Suite.Run r = run(row);
    SimpleBooleanProperty p = new SimpleBooleanProperty(r != null && r.alliances().contains(alliance));
    p.addListener((o, was, now) -> {
      Suite.Run cur = run(row);
      List<SimAlliance> al = new ArrayList<>();
      for (SimAlliance a : SimAlliance.values()) {
        if (a == alliance ? now : cur.alliances().contains(a)) {
          al.add(a);
        }
      }
      if (al.isEmpty()) { // a run needs at least one alliance
        Platform.runLater(table::refresh);
        return;
      }
      set(row, new Suite.Run(cur.scenario(), cur.allAutos(), al, cur.autoDuration()));
    });
    return p;
  }

  private Suite suite() {
    return current == null ? null : ws.suite(current);
  }

  private Suite.Run run(Integer row) {
    Suite s = suite();
    return s == null || row == null || row < 0 || row >= s.runs().size() ? null : s.runs().get(row);
  }

  private void set(int row, Suite.Run run) {
    suite().runs().set(row, run);
    ws.markSuiteDirty(current);
  }

  private void refresh() {
    loading = true;
    List<String> ids = new ArrayList<>(ws.suiteIds());
    if (!ids.equals(suiteList.getItems())) {
      suiteList.getItems().setAll(ids);
    }
    if (current != null && ws.suite(current) != null) {
      suiteList.getSelectionModel().select(current);
    } else {
      current = null;
    }
    suiteList.refresh();
    List<String> choices = new ArrayList<>();
    choices.add(ALL_AUTOS);
    choices.addAll(ws.scenarioIds());
    scenarioChoices.setAll(choices);
    loading = false;
    showCurrent();
  }

  private void showCurrent() {
    loading = true;
    Suite s = suite();
    name.setDisable(s == null);
    name.setText(s == null ? "" : s.getName());
    loading = false;
    int sel = table.getSelectionModel().getSelectedIndex();
    table.getItems().setAll(s == null ? List.of() : IntStream.range(0, s.runs().size()).boxed().toList());
    if (sel >= 0 && sel < table.getItems().size()) {
      table.getSelectionModel().select(sel);
    }
  }

  // ------------------------------------------------------------------ actions

  @FXML
  private void addRun() {
    if (suite() == null) {
      return;
    }
    String first = ws.scenarioIds().isEmpty() ? null : ws.scenarioIds().iterator().next();
    suite().runs().add(first == null
        ? new Suite.Run(null, true, List.of(SimAlliance.BLUE), null)
        : Suite.Run.of(first, SimAlliance.BLUE));
    ws.markSuiteDirty(current);
  }

  @FXML
  private void removeRun() {
    int r = table.getSelectionModel().getSelectedIndex();
    if (suite() != null && r >= 0) {
      suite().runs().remove(r);
      ws.markSuiteDirty(current);
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

  private void move(int d) {
    int r = table.getSelectionModel().getSelectedIndex();
    if (suite() == null || r < 0 || r + d < 0 || r + d >= suite().runs().size()) {
      return;
    }
    List<Suite.Run> runs = suite().runs();
    runs.add(r + d, runs.remove(r));
    ws.markSuiteDirty(current);
    Platform.runLater(() -> table.getSelectionModel().select(r + d));
  }

  @FXML
  private void save() {
    if (current != null) {
      try {
        ws.saveSuite(current);
      } catch (IOException e) {
        Ui.error(ctx.stage, "Save failed", e.getMessage());
      }
    }
  }

  @FXML
  private void newSuite() {
    Ui.ask(ctx.stage, "New suite", "File id (lowercase, digits, - or _):", "new-suite").ifPresent(id -> {
      if (!Workspace.isValidId(id) || ws.suite(id) != null) {
        Ui.warn(ctx.stage, "Invalid id", "Use lowercase letters, digits, - and _, and a name that isn't taken.");
        return;
      }
      current = id;
      ws.putSuite(id, new Suite(id));
    });
  }

  @FXML
  private void deleteSuite() {
    if (current != null && Ui.confirm(ctx.stage, "Delete suite", "Delete suite file " + current + ".json?")) {
      try {
        ws.deleteSuite(current);
        current = null;
      } catch (IOException e) {
        Ui.error(ctx.stage, "Delete failed", e.getMessage());
      }
    }
  }
}
