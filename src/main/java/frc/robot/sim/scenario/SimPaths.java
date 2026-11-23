package frc.robot.sim.scenario;

import java.nio.file.Files;
import java.nio.file.Path;

/**
 * Locates the {@code simulation/} directory and the files in it. Shared by the simulator (which
 * runs with an unpredictable working directory under {@code build/}) and the editor GUI.
 *
 * <p>Resolution order: env var {@code SIM_ROOT} (set by build.gradle to the project directory),
 * then the working directory and its parents.
 */
public final class SimPaths {
  public static final String ENV_VAR = "SIM_ROOT";

  private final Path dir;

  private SimPaths(Path dir) {
    this.dir = dir.toAbsolutePath().normalize();
  }

  /** Paths rooted at an explicit project directory. */
  public static SimPaths forProject(Path projectDir) {
    return new SimPaths(projectDir.resolve("simulation"));
  }

  /** Finds the simulation directory from {@code SIM_ROOT} or by searching up from the cwd. */
  public static SimPaths locate() {
    String env = System.getenv(ENV_VAR);
    if (env != null && !env.isBlank()) {
      return forProject(Path.of(env));
    }
    for (Path p = Path.of("").toAbsolutePath(); p != null; p = p.getParent()) {
      if (Files.isRegularFile(p.resolve("simulation").resolve("field-locations.json"))) {
        return forProject(p);
      }
    }
    return forProject(Path.of(""));
  }

  public Path dir() {
    return dir;
  }

  public Path projectDir() {
    return dir.getParent();
  }

  public Path scenariosDir() {
    return dir.resolve("scenarios");
  }

  public Path scenario(String id) {
    return scenariosDir().resolve(id + ".json");
  }

  public Path suitesDir() {
    return dir.resolve("suites");
  }

  public Path suite(String id) {
    return suitesDir().resolve(id + ".json");
  }

  public Path fieldLocations() {
    return dir.resolve("field-locations.json");
  }

  /** Written by the GUI; tells the next sim launch what to run. Optional. */
  public Path runConfig() {
    return dir.resolve("run.json");
  }

  public Path resultsDir() {
    return dir.resolve("results");
  }

  public Path latestResults() {
    return resultsDir().resolve("latest.json");
  }

  /** Created by the GUI to ask a running sim to finish cleanly. */
  public Path stopFlag() {
    return dir.resolve(".stop");
  }
}
