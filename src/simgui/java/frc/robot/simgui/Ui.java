package frc.robot.simgui;

import java.util.Optional;
import javafx.scene.control.Alert;
import javafx.scene.control.ButtonType;
import javafx.scene.control.Label;
import javafx.scene.control.Spinner;
import javafx.scene.control.TextInputDialog;
import javafx.stage.Window;

/** Small JavaFX helpers shared by the controllers. */
final class Ui {
  private Ui() {}

  /** An editable number spinner (see {@link DoubleSpinner}). */
  static Spinner<Double> spinner(double min, double max, double value, double step) {
    return new DoubleSpinner(min, max, value, step);
  }

  static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }

  /** Compact number text: 0.5, 1, -0.25. */
  static String fmt(double v) {
    if (v == Math.rint(v)) {
      return String.valueOf((long) v);
    }
    return java.math.BigDecimal.valueOf(v).setScale(4, java.math.RoundingMode.HALF_UP).stripTrailingZeros().toPlainString();
  }

  static Label hint(String text) {
    Label l = new Label(text);
    l.getStyleClass().add("hint");
    l.setWrapText(true);
    return l;
  }

  static void info(Window owner, String title, String message) {
    show(owner, Alert.AlertType.INFORMATION, title, message);
  }

  static void warn(Window owner, String title, String message) {
    show(owner, Alert.AlertType.WARNING, title, message);
  }

  static void error(Window owner, String title, String message) {
    show(owner, Alert.AlertType.ERROR, title, message);
  }

  private static void show(Window owner, Alert.AlertType type, String title, String message) {
    Alert a = new Alert(type, message, ButtonType.OK);
    a.initOwner(owner);
    a.setTitle(title);
    a.setHeaderText(null);
    a.getDialogPane().setMinWidth(460);
    a.showAndWait();
  }

  /** OK/Cancel question. */
  static boolean confirm(Window owner, String title, String message) {
    Alert a = new Alert(Alert.AlertType.CONFIRMATION, message, ButtonType.OK, ButtonType.CANCEL);
    a.initOwner(owner);
    a.setTitle(title);
    a.setHeaderText(null);
    return a.showAndWait().filter(b -> b == ButtonType.OK).isPresent();
  }

  /** Yes/No/Cancel question. Empty = cancelled. */
  static Optional<Boolean> yesNoCancel(Window owner, String title, String message) {
    Alert a = new Alert(Alert.AlertType.CONFIRMATION, message, ButtonType.YES, ButtonType.NO, ButtonType.CANCEL);
    a.initOwner(owner);
    a.setTitle(title);
    a.setHeaderText(null);
    return a.showAndWait().flatMap(b -> b == ButtonType.YES ? Optional.of(true) : b == ButtonType.NO ? Optional.of(false) : Optional.empty());
  }

  static Optional<String> ask(Window owner, String title, String prompt, String initial) {
    TextInputDialog d = new TextInputDialog(initial);
    d.initOwner(owner);
    d.setTitle(title);
    d.setHeaderText(null);
    d.setContentText(prompt);
    return d.showAndWait().map(String::trim);
  }
}
