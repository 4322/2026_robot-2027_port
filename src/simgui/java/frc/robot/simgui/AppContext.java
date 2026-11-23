package frc.robot.simgui;

import java.util.function.BiConsumer;
import javafx.stage.Stage;

/**
 * Shared objects handed to every FXML controller (see the controller factory in {@link
 * SimEditorApp}).
 */
final class AppContext {
  final Workspace ws;
  final SimProcess process;
  final Stage stage;

  /** Opens a scenario in the Scenario tab and selects an event: (scenarioId, "segment:index"). */
  BiConsumer<String, String> jumpTo = (id, where) -> {};

  AppContext(Workspace ws, SimProcess process, Stage stage) {
    this.ws = ws;
    this.process = process;
    this.stage = stage;
  }
}
