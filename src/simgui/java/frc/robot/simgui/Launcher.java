package frc.robot.simgui;

import javafx.application.Application;

/**
 * Entry point for {@code ./gradlew simEditor}.
 *
 * <p>JavaFX refuses to start when the main class itself extends {@link Application} and JavaFX
 * is on the classpath (not the module path), so this plain class launches {@link SimEditorApp}.
 */
public final class Launcher {
  private Launcher() {}

  public static void main(String[] args) {
    Application.launch(SimEditorApp.class, args);
  }
}
