package frc.robot.simgui;

import frc.robot.sim.scenario.Pose;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Map;
import java.util.Optional;
import java.util.function.BiFunction;
import java.util.function.UnaryOperator;
import javafx.application.Platform;
import javafx.beans.property.ReadOnlyObjectWrapper;
import javafx.beans.property.ReadOnlyStringWrapper;
import javafx.fxml.FXML;
import javafx.scene.control.Label;
import javafx.scene.control.Spinner;
import javafx.scene.control.TableColumn;
import javafx.scene.control.TableView;
import javafx.scene.control.cell.TextFieldTableCell;
import javafx.scene.layout.StackPane;
import javafx.scene.paint.Color;
import javafx.util.converter.DoubleStringConverter;

/**
 * Field Locations tab: edits {@code field-locations.json} (named poses and the editor's path
 * preview settings). Renaming a location updates every scenario that uses it. Layout:
 * locations.fxml.
 */
public final class LocationsController {
  private static final Color LOCATION_COLOR = Color.web("#00897b");

  private final AppContext ctx;
  private final Workspace ws;
  private FieldCanvas field;
  private boolean loading;

  @FXML private StackPane fieldHolder;
  @FXML private TableView<String> table;
  @FXML private TableColumn<String, String> colName;
  @FXML private TableColumn<String, Double> colX;
  @FXML private TableColumn<String, Double> colY;
  @FXML private TableColumn<String, Double> colDeg;
  @FXML private TableColumn<String, String> colUsed;
  @FXML private Spinner<Double> maxSpeed;
  @FXML private Spinner<Double> maxTurn;
  @FXML private Label status;

  public LocationsController(AppContext ctx) {
    this.ctx = ctx;
    this.ws = ctx.ws;
  }

  @FXML
  private void initialize() {
    field = new FieldCanvas(ws::field);
    field.setScene(this::scene);
    field.setListener(new FieldCanvas.Listener() {
      @Override
      public void selected(Object key) {
        table.getSelectionModel().select((String) key);
      }

      @Override
      public void markerMoved(Object key, double x, double y, boolean finished) {
        update((String) key, p -> new Pose(x, y, p.deg()));
      }

      @Override
      public void markerRotated(Object key, double deg, boolean finished) {
        update((String) key, p -> p.withDeg(deg));
      }
    });
    fieldHolder.getChildren().add(field);

    colName.setCellValueFactory(c -> new ReadOnlyStringWrapper(c.getValue()));
    colX.setCellValueFactory(c -> new ReadOnlyObjectWrapper<>(pose(c.getValue()).x()));
    colY.setCellValueFactory(c -> new ReadOnlyObjectWrapper<>(pose(c.getValue()).y()));
    colDeg.setCellValueFactory(c -> new ReadOnlyObjectWrapper<>(pose(c.getValue()).deg()));
    colUsed.setCellValueFactory(c -> new ReadOnlyStringWrapper(String.valueOf(ws.scenariosUsingLocation(c.getValue()).size())));
    editable(colX, (p, v) -> p.withX(v));
    editable(colY, (p, v) -> p.withY(v));
    editable(colDeg, (p, v) -> p.withDeg(v));
    colName.setEditable(false);
    colUsed.setEditable(false);
    table.getSelectionModel().selectedItemProperty().addListener((o, a, b) -> field.redraw());

    maxSpeed.valueProperty().addListener((o, a, v) -> {
      if (!loading) {
        ws.field().setPreviewMaxSpeedMps(v);
        ws.markFieldDirty();
      }
    });
    maxTurn.valueProperty().addListener((o, a, v) -> {
      if (!loading) {
        ws.field().setPreviewMaxTurnDegPerSec(v);
        ws.markFieldDirty();
      }
    });

    ws.addListener(() -> Platform.runLater(this::refresh));
    refresh();
  }

  private void editable(TableColumn<String, Double> col, BiFunction<Pose, Double, Pose> apply) {
    col.setCellFactory(TextFieldTableCell.forTableColumn(new DoubleStringConverter() {
      @Override
      public Double fromString(String s) {
        try {
          return super.fromString(s);
        } catch (RuntimeException e) {
          return null;
        }
      }
    }));
    col.setOnEditCommit(e -> {
      if (e.getNewValue() != null) {
        update(e.getRowValue(), p -> apply.apply(p, e.getNewValue()));
      } else {
        table.refresh();
      }
    });
  }

  private Pose pose(String name) {
    Pose p = ws.field().locations().get(name);
    return p == null ? new Pose(0, 0, 0) : p;
  }

  private void update(String name, UnaryOperator<Pose> change) {
    Pose p = ws.field().locations().get(name);
    if (p != null) {
      ws.field().locations().put(name, change.apply(p));
      ws.markFieldDirty();
    }
  }

  private void refresh() {
    loading = true;
    maxSpeed.getValueFactory().setValue(ws.field().getPreviewMaxSpeedMps());
    maxTurn.getValueFactory().setValue(ws.field().getPreviewMaxTurnDegPerSec());
    loading = false;
    String sel = table.getSelectionModel().getSelectedItem();
    List<String> names = new ArrayList<>(ws.field().locations().keySet());
    if (!names.equals(table.getItems())) {
      table.getItems().setAll(names);
    }
    if (sel != null && names.contains(sel)) {
      table.getSelectionModel().select(sel);
    }
    table.refresh();
    status.getStyleClass().removeAll("status-warn", "hint");
    status.setText(ws.isFieldDirty() ? "field-locations.json has unsaved changes" : "field-locations.json saved");
    status.getStyleClass().add(ws.isFieldDirty() ? "status-warn" : "hint");
    field.redraw();
  }

  private FieldCanvas.Scene scene() {
    List<FieldCanvas.Marker> markers = new ArrayList<>();
    String sel = table.getSelectionModel().getSelectedItem();
    for (Map.Entry<String, Pose> e : ws.field().locations().entrySet()) {
      markers.add(new FieldCanvas.Marker(e.getKey(), e.getValue(), e.getKey(), LOCATION_COLOR, e.getKey().equals(sel), true, false));
    }
    return new FieldCanvas.Scene(markers, List.of(), List.of(), null);
  }

  // ------------------------------------------------------------------ actions

  @FXML
  private void add() {
    askName("New location", "NewLocation").ifPresent(name -> {
      ws.field().locations().put(name, new Pose(2, 2, 0));
      ws.markFieldDirty();
      Platform.runLater(() -> table.getSelectionModel().select(name));
    });
  }

  @FXML
  private void rename() {
    String old = table.getSelectionModel().getSelectedItem();
    if (old == null) {
      return;
    }
    Optional<String> name = askName("Rename \"" + old + "\"", old);
    if (name.isEmpty() || name.get().equals(old)) {
      return;
    }
    List<String> users = ws.scenariosUsingLocation(old);
    if (!users.isEmpty() && !Ui.confirm(ctx.stage, "Rename location",
        "Also update " + users.size() + " scenario(s) that use it?\n" + String.join(", ", users)
            + "\n\nThey'll be marked unsaved; use Save all.")) {
      return;
    }
    ws.renameLocation(old, name.get());
    Platform.runLater(() -> table.getSelectionModel().select(name.get()));
  }

  @FXML
  private void delete() {
    String name = table.getSelectionModel().getSelectedItem();
    if (name == null) {
      return;
    }
    List<String> users = ws.scenariosUsingLocation(name);
    if (!users.isEmpty()) {
      Ui.warn(ctx.stage, "Location in use", "\"" + name + "\" is used by: " + String.join(", ", users)
          + "\nChange those events first.");
      return;
    }
    ws.field().locations().remove(name);
    ws.markFieldDirty();
  }

  @FXML
  private void save() {
    try {
      ws.saveField();
    } catch (IOException e) {
      Ui.error(ctx.stage, "Save failed", e.getMessage());
    }
  }

  private Optional<String> askName(String title, String initial) {
    while (true) {
      Optional<String> s = Ui.ask(ctx.stage, title, "Name (letters, digits, _):", initial);
      if (s.isEmpty()) {
        return s;
      }
      String n = s.get();
      if (!n.matches("[A-Za-z][A-Za-z0-9_]*")) {
        Ui.warn(ctx.stage, "Invalid name", "Use letters, digits and _ only, starting with a letter.");
      } else if (ws.field().locations().containsKey(n) && !n.equals(initial)) {
        Ui.warn(ctx.stage, "Already exists", "\"" + n + "\" already exists.");
      } else {
        return s;
      }
    }
  }
}
