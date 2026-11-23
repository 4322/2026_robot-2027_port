package frc.robot.simgui;

import frc.robot.sim.scenario.SimPaths;
import java.io.IOException;
import java.nio.file.Path;
import java.util.List;
import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Parent;
import javafx.scene.Scene;
import javafx.stage.Screen;
import javafx.stage.Stage;

/**
 * Simulator scenario editor and runner. Launch with {@code ./gradlew simEditor}.
 *
 * <p>Edits the JSON files in {@code simulation/} (scenarios, field locations, suites) and runs the
 * robot simulation, scripted or with a real gamepad. Robot output is viewed in AdvantageScope.
 *
 * <p>Layout lives in the FXML files and styling in {@code editor.css}, both under {@code
 * src/simgui/resources/frc/robot/simgui}. Each FXML file names its controller class.
 */
public final class SimEditorApp extends Application {
  private AppContext ctx;

  @Override
  public void start(Stage stage) throws IOException {
    List<String> args = getParameters().getRaw();
    SimPaths paths = args.isEmpty() ? SimPaths.locate() : SimPaths.forProject(Path.of(args.get(0)));
    ctx = new AppContext(new Workspace(paths), new SimProcess(paths), stage);

    Parent root = load("main.fxml", ctx);
    var bounds = Screen.getPrimary().getVisualBounds();
    Scene scene = new Scene(root, Math.min(1600, bounds.getWidth() - 60), Math.min(950, bounds.getHeight() - 60));
    scene.getStylesheets().add(SimEditorApp.class.getResource("editor.css").toExternalForm());
    stage.setScene(scene);
    stage.setTitle("Simulator Scenario Editor");
    stage.show();
  }

  /**
   * Loads an FXML file next to this class. Controllers (including those of {@code fx:include}d
   * files) are built with a constructor taking {@link AppContext} when they have one.
   */
  static Parent load(String fxml, AppContext ctx) throws IOException {
    FXMLLoader loader = new FXMLLoader(SimEditorApp.class.getResource(fxml));
    loader.setControllerFactory(type -> {
      try {
        try {
          return type.getDeclaredConstructor(AppContext.class).newInstance(ctx);
        } catch (NoSuchMethodException e) {
          return type.getDeclaredConstructor().newInstance();
        }
      } catch (ReflectiveOperationException e) {
        throw new IllegalStateException("Cannot create controller " + type.getName(), e);
      }
    });
    return loader.load();
  }

  @Override
  public void stop() {
    if (ctx != null) {
      ctx.process.kill();
    }
  }
}
