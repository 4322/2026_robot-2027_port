package frc.robot.simgui;

import frc.robot.sim.scenario.SimPaths;
import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;
import java.util.function.Consumer;
import java.util.function.IntConsumer;

/**
 * Runs the robot simulation ({@code gradlew run}) as a child process, streaming its output.
 *
 * <p>Stopping first asks the simulator to finish cleanly (it polls {@code simulation/.stop}, then
 * writes results and exits); if it hasn't exited after a few seconds the process tree is killed.
 */
final class SimProcess {
  private static final long GRACEFUL_STOP_MS = 6000;

  private final SimPaths paths;
  private Process process;

  SimProcess(SimPaths paths) {
    this.paths = paths;
  }

  boolean isRunning() {
    return process != null && process.isAlive();
  }

  /**
   * Starts {@code gradlew run}.
   *
   * @param headless add {@code -PsimHeadless} (no sim GUI window)
   * @param offline add {@code --offline}
   * @param output receives each output line (on a background thread)
   * @param onExit receives the exit code (on a background thread)
   */
  void start(boolean headless, boolean offline, Consumer<String> output, IntConsumer onExit) throws IOException {
    if (isRunning()) {
      throw new IllegalStateException("simulation already running");
    }
    Files.deleteIfExists(paths.stopFlag());
    Path project = paths.projectDir();
    boolean windows = System.getProperty("os.name").toLowerCase().contains("win");
    List<String> cmd = new ArrayList<>();
    if (windows) {
      cmd.add("cmd.exe");
      cmd.add("/c");
      cmd.add(project.resolve("gradlew.bat").toString());
    } else {
      cmd.add(project.resolve("gradlew").toString());
    }
    cmd.add("run");
    if (headless) {
      cmd.add("-PsimHeadless");
    }
    if (offline) {
      cmd.add("--offline");
    }
    ProcessBuilder pb = new ProcessBuilder(cmd).directory(project.toFile()).redirectErrorStream(true);
    pb.environment().put(SimPaths.ENV_VAR, project.toString());
    // Gradle must run on a JDK its Groovy can handle; reuse the one running this editor if none is set.
    if (System.getenv("JAVA_HOME") == null || System.getenv("JAVA_HOME").isBlank()) {
      pb.environment().put("JAVA_HOME", System.getProperty("java.home"));
    }
    output.accept("> " + String.join(" ", cmd));
    process = pb.start();
    Process p = process;
    Thread reader = new Thread(() -> {
      try (BufferedReader r = new BufferedReader(new InputStreamReader(p.getInputStream(), StandardCharsets.UTF_8))) {
        String line;
        while ((line = r.readLine()) != null) {
          output.accept(line);
        }
      } catch (IOException ignored) {
        // process ended
      }
      try {
        onExit.accept(p.waitFor());
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      }
    }, "sim-output");
    reader.setDaemon(true);
    reader.start();
  }

  /** Asks the sim to stop cleanly, then force-kills it if needed. Non-blocking. */
  void stop(Consumer<String> output) {
    Process p = process;
    if (p == null || !p.isAlive()) {
      return;
    }
    try {
      Files.writeString(paths.stopFlag(), "stop\n");
      output.accept("[editor] Asked the simulator to stop...");
    } catch (IOException e) {
      output.accept("[editor] Could not write stop flag: " + e.getMessage());
    }
    Thread killer = new Thread(() -> {
      try {
        if (!p.waitFor(GRACEFUL_STOP_MS, TimeUnit.MILLISECONDS)) {
          output.accept("[editor] Simulator didn't stop; killing the process.");
          p.descendants().forEach(ProcessHandle::destroyForcibly);
          p.destroyForcibly();
        }
        Files.deleteIfExists(paths.stopFlag());
      } catch (InterruptedException e) {
        Thread.currentThread().interrupt();
      } catch (IOException ignored) {
        // best effort
      }
    }, "sim-stop");
    killer.setDaemon(true);
    killer.start();
  }

  /** Kills the process tree immediately (editor closing). */
  void kill() {
    Process p = process;
    if (p != null && p.isAlive()) {
      p.descendants().forEach(ProcessHandle::destroyForcibly);
      p.destroyForcibly();
    }
    try {
      Files.deleteIfExists(paths.stopFlag());
    } catch (IOException ignored) {
      // best effort
    }
  }
}
