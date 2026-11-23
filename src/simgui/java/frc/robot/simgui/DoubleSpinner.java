package frc.robot.simgui;

import javafx.beans.NamedArg;
import javafx.scene.control.Spinner;
import javafx.scene.control.SpinnerValueFactory;
import javafx.util.StringConverter;

/**
 * An editable number spinner. Arrows update live; typed text commits on Enter or when focus
 * leaves. Usable from FXML:
 *
 * <pre>{@code <DoubleSpinner min="0" max="600" initialValue="20" step="1"/>}</pre>
 */
public class DoubleSpinner extends Spinner<Double> {
  public DoubleSpinner(
      @NamedArg("min") double min,
      @NamedArg("max") double max,
      @NamedArg("initialValue") double initialValue,
      @NamedArg(value = "step", defaultValue = "1") double step) {
    var factory = new SpinnerValueFactory.DoubleSpinnerValueFactory(min, max, Ui.clamp(initialValue, min, max), step);
    factory.setConverter(new StringConverter<>() {
      @Override
      public String toString(Double v) {
        return v == null ? "" : Ui.fmt(v);
      }

      @Override
      public Double fromString(String s) {
        try {
          return Ui.clamp(Double.parseDouble(s.trim()), min, max);
        } catch (NumberFormatException e) {
          return factory.getValue();
        }
      }
    });
    setValueFactory(factory);
    setEditable(true);
    setPrefWidth(110);
    getEditor().setText(factory.getConverter().toString(factory.getValue()));
    // Enter commits typed text (built in); also commit when focus leaves so a typed value isn't lost.
    focusedProperty().addListener((obs, was, now) -> {
      if (!now) {
        factory.setValue(factory.getConverter().fromString(getEditor().getText()));
        getEditor().setText(factory.getConverter().toString(factory.getValue()));
      }
    });
  }

  /** Sets the value (clamped to the range). */
  public void set(double value) {
    getValueFactory().setValue(value);
  }
}
