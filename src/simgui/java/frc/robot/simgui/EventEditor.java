package frc.robot.simgui;

import frc.robot.sim.scenario.Pose;
import frc.robot.sim.scenario.ProbeCatalog;
import frc.robot.sim.scenario.ScenarioEvent;
import frc.robot.sim.scenario.ScenarioEvent.ButtonAction;
import frc.robot.sim.scenario.ScenarioEvent.Frame;
import frc.robot.sim.scenario.ScenarioEvent.GamepadButton;
import frc.robot.sim.scenario.ScenarioEvent.PovDirection;
import frc.robot.sim.scenario.ScenarioEvent.TriggerSide;
import java.util.List;
import java.util.function.Consumer;
import java.util.function.Function;
import java.util.function.Supplier;
import javafx.beans.value.ObservableValue;
import javafx.scene.Node;
import javafx.scene.canvas.Canvas;
import javafx.scene.canvas.GraphicsContext;
import javafx.scene.control.Button;
import javafx.scene.control.CheckBox;
import javafx.scene.control.ComboBox;
import javafx.scene.control.Label;
import javafx.scene.control.RadioButton;
import javafx.scene.control.Spinner;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleGroup;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.GridPane;
import javafx.scene.layout.HBox;
import javafx.scene.layout.VBox;
import javafx.scene.paint.Color;

/**
 * Type-specific form for one {@link ScenarioEvent}. Every edit immediately builds a replacement
 * event and hands it to the change callback. Built in code because the fields depend on the event
 * type; styling comes from the {@code .event-form} rules in editor.css.
 */
final class EventEditor extends VBox {
  private final Supplier<List<String>> locationNames;
  /** Looks up a named location's pose (BLUE frame) for "Detach". */
  private final Function<String, Pose> locationPose;
  private ScenarioEvent event;
  private Consumer<ScenarioEvent> onChange = e -> {};
  /** True while the form is being filled from an event (suppresses change callbacks). */
  private boolean loading;
  private GridPane form;
  private int row;

  EventEditor(Supplier<List<String>> locationNames, Function<String, Pose> locationPose) {
    this.locationNames = locationNames;
    this.locationPose = locationPose;
    setSpacing(8);
    showNothing();
  }

  void showNothing() {
    event = null;
    getChildren().setAll(Ui.hint("Select an event in the table, or add one."));
  }

  void edit(ScenarioEvent ev, Consumer<ScenarioEvent> onChange) {
    this.event = ev;
    this.onChange = onChange;
    loading = true;
    form = new GridPane();
    form.getStyleClass().add("event-form");
    row = 0;
    Label title = new Label("Event: " + ev.type());
    title.getStyleClass().add("event-type-title");
    getChildren().setAll(title, form);

    Spinner<Double> wait = Ui.spinner(0, 600, ev.waitSec(), 0.1);
    addRow("Wait (s after previous)", wait, () -> emit(event.withWait(wait.getValue())));
    TextField label = new TextField(ev.label() == null ? "" : ev.label());
    label.setPromptText("optional, shown as Sim/EventName");
    addRow("Label", label, () -> emit(event.withLabel(label.getText().isBlank() ? null : label.getText())));
    switch (ev) {
      case ScenarioEvent.SetPose p -> buildSetPose(p);
      case ScenarioEvent.Button b -> buildButton(b);
      case ScenarioEvent.Pov p -> buildPov(p);
      case ScenarioEvent.Trigger t -> buildTrigger(t);
      case ScenarioEvent.Drive d -> buildDrive(d);
      case ScenarioEvent.Turn t -> buildTurn(t);
      case ScenarioEvent.GameData g -> buildGameData(g);
      case ScenarioEvent.Expect x -> buildExpect(x);
      case ScenarioEvent.StopJoysticks s -> note("Centres both sticks.");
      case ScenarioEvent.End e -> note("Ends the segment. Must be the last event.");
    }
    loading = false;
  }

  /** Re-shows the current event (e.g. after it was changed by dragging on the field). */
  void refresh(ScenarioEvent ev) {
    if (event != null && ev != null && ev.getClass() == event.getClass()) {
      edit(ev, onChange);
    }
  }

  private void emit(ScenarioEvent replacement) {
    if (loading) {
      return;
    }
    event = replacement;
    onChange.accept(replacement);
  }

  // ------------------------------------------------------------------ per-type forms

  private void buildSetPose(ScenarioEvent.SetPose p) {
    PoseFields pf = new PoseFields(p.location(), p.pose(), p.frame());
    pf.onChange = () -> emit(new ScenarioEvent.SetPose(event.waitSec(), event.label(), pf.location(), pf.pose(), pf.frame()));
    pf.addRows();
  }

  private void buildButton(ScenarioEvent.Button b) {
    ComboBox<GamepadButton> button = combo(GamepadButton.values(), b.button());
    ComboBox<ButtonAction> action = combo(ButtonAction.values(), b.action());
    Runnable r = () -> emit(new ScenarioEvent.Button(event.waitSec(), event.label(), button.getValue(), action.getValue()));
    addRow("Button", button, r);
    addRow("Action", action, r);
    note("press = down for one robot loop; hold = down until a release.");
  }

  private void buildPov(ScenarioEvent.Pov p) {
    ComboBox<PovDirection> dir = combo(PovDirection.values(), p.direction());
    ComboBox<ButtonAction> action = combo(ButtonAction.values(), p.action());
    Runnable r = () -> emit(new ScenarioEvent.Pov(event.waitSec(), event.label(), dir.getValue(), action.getValue()));
    addRow("Direction", dir, r);
    addRow("Action", action, r);
  }

  private void buildTrigger(ScenarioEvent.Trigger t) {
    ComboBox<TriggerSide> side = combo(TriggerSide.values(), t.side());
    Spinner<Double> value = Ui.spinner(0, 1, t.value(), 0.1);
    Runnable r = () -> emit(new ScenarioEvent.Trigger(event.waitSec(), event.label(), side.getValue(), value.getValue()));
    addRow("Trigger", side, r);
    addRow("Value (0-1)", value, r);
  }

  private void buildDrive(ScenarioEvent.Drive d) {
    Spinner<Double> x = Ui.spinner(-1, 1, d.x(), 0.05);
    Spinner<Double> y = Ui.spinner(-1, 1, d.y(), 0.05);
    StickPad pad = new StickPad(d.x(), d.y());
    Runnable fromSpinners = () -> {
      pad.set(x.getValue(), y.getValue());
      emit(new ScenarioEvent.Drive(event.waitSec(), event.label(), x.getValue(), y.getValue()));
    };
    addRow("Forward (x)", x, fromSpinners);
    addRow("Left (y)", y, fromSpinners);
    pad.onChange = (px, py) -> {
      loading = true;
      x.getValueFactory().setValue(px);
      y.getValueFactory().setValue(py);
      loading = false;
      emit(new ScenarioEvent.Drive(event.waitSec(), event.label(), px, py));
    };
    addRow("Stick (drag)", pad, null);
    note("Driver-relative: forward = away from your driver station. You can also drag the arrow tip on the field.");
  }

  private void buildTurn(ScenarioEvent.Turn t) {
    Spinner<Double> rate = Ui.spinner(-1, 1, t.rate(), 0.05);
    addRow("Rate (+ = CCW)", rate, () -> emit(new ScenarioEvent.Turn(event.waitSec(), event.label(), rate.getValue())));
  }

  private void buildGameData(ScenarioEvent.GameData g) {
    ComboBox<String> value = combo(new String[] {"", "B", "R"}, g.value());
    addRow("Message", value, () -> emit(new ScenarioEvent.GameData(event.waitSec(), event.label(), value.getValue())));
    note("B = blue hub inactive first, R = red hub inactive first.");
  }

  private void buildExpect(ScenarioEvent.Expect x) {
    ComboBox<String> probe = combo(ProbeCatalog.names().toArray(String[]::new), x.probe());
    probe.setEditable(true);
    addRow("Probe", probe, () -> {
      String name = probe.getValue();
      // Switching probe resets the comparison fields to something valid for its kind.
      ScenarioEvent.Expect fresh = defaultExpect(name, ProbeCatalog.find(name), event.waitSec(), event.label(), x.within());
      emit(fresh);
      edit(fresh, onChange);
    });
    ProbeCatalog.Probe info = ProbeCatalog.find(x.probe());
    note(info == null ? "Unknown probe" : info.description());

    CheckBox hasWithin = new CheckBox("Allow up to (s)");
    hasWithin.setSelected(x.within() != null);
    Spinner<Double> within = Ui.spinner(0, 600, x.within() == null ? 1.0 : x.within(), 0.1);
    within.disableProperty().bind(hasWithin.selectedProperty().not());

    if (x.isPoseCheck()) {
      PoseFields pf = new PoseFields(x.location(), x.pose(), x.frame());
      Spinner<Double> tol = Ui.spinner(0.01, 10, x.toleranceOrDefault(), 0.05);
      CheckBox hasHeading = new CheckBox("Heading within (deg)");
      hasHeading.setSelected(x.headingTolDeg() != null);
      Spinner<Double> heading = Ui.spinner(0.5, 180, x.headingTolDeg() == null ? 10 : x.headingTolDeg(), 1);
      heading.disableProperty().bind(hasHeading.selectedProperty().not());
      Runnable r = () -> emit(new ScenarioEvent.Expect(event.waitSec(), event.label(), "pose", null, null, null,
          pf.location(), pf.pose(), pf.frame(), tol.getValue(), hasHeading.isSelected() ? heading.getValue() : null,
          hasWithin.isSelected() ? within.getValue() : null));
      pf.onChange = r;
      pf.addRows();
      addRow("Tolerance (m)", tol, r);
      addRow(hasHeading, heading, r);
      addRow(hasWithin, within, r);
    } else {
      ProbeCatalog.Kind kind = info == null ? ProbeCatalog.Kind.ENUM : info.kind();
      String[] values = kind == ProbeCatalog.Kind.BOOLEAN ? new String[] {"true", "false"}
          : info == null ? new String[0] : info.values().toArray(String[]::new);
      ComboBox<String> equals = combo(values, x.equalsValue());
      equals.setEditable(kind != ProbeCatalog.Kind.BOOLEAN);
      CheckBox hasEquals = new CheckBox("Equals");
      hasEquals.setSelected(x.equalsValue() != null);
      equals.disableProperty().bind(hasEquals.selectedProperty().not());
      CheckBox hasMin = new CheckBox("Min");
      hasMin.setSelected(x.min() != null);
      Spinner<Double> min = Ui.spinner(-1e6, 1e6, x.min() == null ? 0 : x.min(), 0.1);
      min.disableProperty().bind(hasMin.selectedProperty().not());
      CheckBox hasMax = new CheckBox("Max");
      hasMax.setSelected(x.max() != null);
      Spinner<Double> max = Ui.spinner(-1e6, 1e6, x.max() == null ? 0 : x.max(), 0.1);
      max.disableProperty().bind(hasMax.selectedProperty().not());
      Runnable r = () -> {
        String eq = equals.getValue();
        emit(new ScenarioEvent.Expect(event.waitSec(), event.label(), x.probe(),
            hasEquals.isSelected() && eq != null && !eq.isBlank() ? eq : null,
            hasMin.isSelected() ? min.getValue() : null, hasMax.isSelected() ? max.getValue() : null,
            null, null, Frame.ALLIANCE, null, null, hasWithin.isSelected() ? within.getValue() : null));
      };
      addRow(hasEquals, equals, r);
      if (kind == ProbeCatalog.Kind.NUMBER || x.min() != null || x.max() != null) {
        addRow(hasMin, min, r);
        addRow(hasMax, max, r);
      }
      addRow(hasWithin, within, r);
    }
    note("\"Allow up to\": passes if true at any point in that many seconds; otherwise checked once.");
  }

  /** A reasonable starting expect for a probe. */
  static ScenarioEvent.Expect defaultExpect(String name, ProbeCatalog.Probe info, double wait, String label, Double within) {
    ProbeCatalog.Kind kind = info == null ? ProbeCatalog.Kind.ENUM : info.kind();
    return switch (kind) {
      case POSE -> new ScenarioEvent.Expect(wait, label, "pose", null, null, null, null, new Pose(2, 2, 0), Frame.ALLIANCE, 0.25, null, within);
      case NUMBER -> new ScenarioEvent.Expect(wait, label, name, null, 0.0, null, null, null, Frame.ALLIANCE, null, null, within);
      case BOOLEAN -> new ScenarioEvent.Expect(wait, label, name, "true", null, null, null, null, Frame.ALLIANCE, null, null, within);
      case ENUM -> new ScenarioEvent.Expect(wait, label, name,
          info != null && !info.values().isEmpty() ? info.values().get(0) : "", null, null, null, null, Frame.ALLIANCE, null, null, within);
    };
  }

  // ------------------------------------------------------------------ pose fields (setPose + pose expect)

  /** "Named location" vs "explicit x/y/deg" plus the alliance frame. */
  private final class PoseFields {
    final ToggleGroup group = new ToggleGroup();
    final RadioButton named = new RadioButton("Named location");
    final RadioButton explicit = new RadioButton("Explicit pose");
    final ComboBox<String> location = new ComboBox<>();
    final Spinner<Double> x;
    final Spinner<Double> y;
    final Spinner<Double> deg;
    final ComboBox<String> frame = new ComboBox<>();
    final Button detach = new Button("Copy to explicit pose");
    Runnable onChange = () -> {};

    PoseFields(String loc, Pose pose, Frame f) {
      Pose start = pose != null ? pose : new Pose(2, 2, 0);
      x = Ui.spinner(-5, 25, start.x(), 0.05);
      y = Ui.spinner(-5, 15, start.y(), 0.05);
      deg = Ui.spinner(-180, 180, start.deg(), 5);
      named.setToggleGroup(group);
      explicit.setToggleGroup(group);
      location.getItems().setAll(locationNames.get());
      boolean isNamed = loc != null;
      (isNamed ? named : explicit).setSelected(true);
      if (isNamed) {
        location.setValue(loc);
      } else if (!location.getItems().isEmpty()) {
        location.setValue(location.getItems().get(0));
      }
      named.setDisable(location.getItems().isEmpty());
      frame.getItems().setAll("Alliance-relative (flip for RED)", "Absolute (never flip)");
      frame.getSelectionModel().select(f == Frame.ABSOLUTE ? 1 : 0);
      location.disableProperty().bind(named.selectedProperty().not());
      detach.disableProperty().bind(named.selectedProperty().not());
      x.disableProperty().bind(explicit.selectedProperty().not());
      y.disableProperty().bind(explicit.selectedProperty().not());
      deg.disableProperty().bind(explicit.selectedProperty().not());
      detach.setOnAction(e -> {
        Pose p = locationPose.apply(location.getValue());
        if (p != null) {
          loading = true;
          x.getValueFactory().setValue(p.x());
          y.getValueFactory().setValue(p.y());
          deg.getValueFactory().setValue(p.deg());
          explicit.setSelected(true);
          loading = false;
          onChange.run();
        }
      });
    }

    void addRows() {
      Runnable r = () -> onChange.run();
      HBox namedRow = new HBox(6, location, detach);
      addRow(named, namedRow, r);
      watch(location.valueProperty(), r);
      addRow(explicit, Ui.hint("drag the robot on the field, or type below"), r);
      addRow("x (m)", x, r);
      addRow("y (m)", y, r);
      addRow("heading (deg)", deg, r);
      addRow("Frame", frame, r);
    }

    String location() {
      return named.isSelected() ? location.getValue() : null;
    }

    Pose pose() {
      return explicit.isSelected() ? new Pose(x.getValue(), y.getValue(), deg.getValue()) : null;
    }

    Frame frame() {
      return frame.getSelectionModel().getSelectedIndex() == 1 ? Frame.ABSOLUTE : Frame.ALLIANCE;
    }
  }

  // ------------------------------------------------------------------ stick pad

  /** A small circle you drag to set a stick vector (clamped to the unit circle). Up = forward. */
  static final class StickPad extends Canvas {
    interface Listener {
      void changed(double x, double y);
    }

    private double sx;
    private double sy;
    Listener onChange = (x, y) -> {};

    StickPad(double x, double y) {
      super(120, 120);
      sx = x;
      sy = y;
      setOnMousePressed(this::update);
      setOnMouseDragged(this::update);
      draw();
    }

    void set(double x, double y) {
      sx = x;
      sy = y;
      draw();
    }

    private void update(MouseEvent e) {
      double r = getWidth() / 2 - 6;
      double c = getWidth() / 2;
      double[] v = clampStick((c - e.getY()) / r, (c - e.getX()) / r); // up = +x, left = +y
      sx = v[0];
      sy = v[1];
      draw();
      onChange.changed(sx, sy);
    }

    private void draw() {
      GraphicsContext g = getGraphicsContext2D();
      double w = getWidth();
      double r = w / 2 - 6;
      double c = w / 2;
      g.clearRect(0, 0, w, w);
      g.setFill(Color.web("#1b1e22"));
      g.fillOval(c - r, c - r, 2 * r, 2 * r);
      g.setStroke(Color.web("#4a525c"));
      g.setLineWidth(1);
      g.strokeOval(c - r, c - r, 2 * r, 2 * r);
      g.strokeLine(c - r, c, c + r, c);
      g.strokeLine(c, c - r, c, c + r);
      double px = c - sy * r;
      double py = c - sx * r;
      g.setStroke(Color.web("#4fc3f7"));
      g.setLineWidth(2.5);
      g.strokeLine(c, c, px, py);
      g.setFill(Color.web("#4fc3f7"));
      g.fillOval(px - 6, py - 6, 12, 12);
    }
  }

  /** Clamps a stick vector to the unit circle and rounds to 0.05. */
  static double[] clampStick(double x, double y) {
    double len = Math.hypot(x, y);
    if (len > 1) {
      x /= len;
      y /= len;
    }
    x = Math.round(x * 20) / 20.0;
    y = Math.round(y * 20) / 20.0;
    if (Math.hypot(x, y) > 1) { // rounding can push it just outside
      x = Math.signum(x) * Math.floor(Math.abs(x) * 20) / 20.0;
      y = Math.signum(y) * Math.floor(Math.abs(y) * 20) / 20.0;
    }
    return new double[] {x + 0.0, y + 0.0};
  }

  // ------------------------------------------------------------------ layout helpers

  private void addRow(String label, Node field, Runnable onEdit) {
    Label l = new Label(label);
    l.getStyleClass().add("form-label");
    addRow(l, field, onEdit);
  }

  private void addRow(Node label, Node field, Runnable onEdit) {
    form.add(label, 0, row);
    form.add(field, 1, row);
    row++;
    listen(label, onEdit);
    listen(field, onEdit);
  }

  private void note(String text) {
    Label l = Ui.hint(text);
    l.setMaxWidth(380);
    form.add(l, 0, row++, 2, 1);
  }

  private <T> ComboBox<T> combo(T[] items, T selected) {
    ComboBox<T> c = new ComboBox<>();
    c.getItems().setAll(items);
    c.setValue(selected);
    return c;
  }

  private void watch(ObservableValue<?> value, Runnable onEdit) {
    value.addListener((obs, old, now) -> {
      if (!loading) {
        onEdit.run();
      }
    });
  }

  private void listen(Node n, Runnable onEdit) {
    if (onEdit == null) {
      return;
    }
    switch (n) {
      case Spinner<?> s -> watch(s.valueProperty(), onEdit);
      case ComboBox<?> c -> watch(c.valueProperty(), onEdit);
      case CheckBox c -> watch(c.selectedProperty(), onEdit);
      case RadioButton rb -> watch(rb.selectedProperty(), onEdit);
      case TextField tf -> watch(tf.textProperty(), onEdit);
      default -> {}
    }
  }
}
