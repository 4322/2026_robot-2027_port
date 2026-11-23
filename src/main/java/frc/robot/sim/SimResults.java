package frc.robot.sim;

import frc.robot.sim.scenario.RunReport;
import frc.robot.sim.scenario.ScenarioJson;
import frc.robot.sim.scenario.SimAlliance;
import java.io.IOException;
import java.nio.file.Path;
import java.util.Locale;
import org.littletonrobotics.junction.Logger;

/** Collects pass/fail results for a run, logs them, and writes the JSON report. */
final class SimResults {
  private final RunReport report = new RunReport();
  private RunReport.Test current;
  private int failures;

  /** A problem with the run as a whole (bad config, missing scenario file, ...). */
  void runError(String message) {
    System.err.println("[Sim] ERROR: " + message);
    report.errors().add(message);
    failures++;
    Logger.recordOutput("Sim/Results/Failures", failures);
  }

  void beginTest(String name, String scenarioId, SimAlliance alliance) {
    current = new RunReport.Test(name, scenarioId, alliance);
    report.tests().add(current);
  }

  /** A problem with the current test that isn't a specific check. */
  void testError(String message) {
    if (current == null) {
      runError(message);
      return;
    }
    System.err.println("[Sim] FAIL " + current.name() + ": " + message);
    current.errors().add(message);
    failures++;
    Logger.recordOutput("Sim/Results/Failures", failures);
  }

  void check(String segment, int eventIndex, String label, boolean passed, String actual, double simTime) {
    current.checks().add(new RunReport.Check(segment, eventIndex, label, passed, actual, simTime));
    Logger.recordOutput("Sim/Expect/" + label, passed);
    System.out.printf(
        Locale.ROOT, "[Sim] %s %s @%.2fs: %s (%s)%n", passed ? "PASS" : "FAIL", current.name(), simTime, label, actual);
    if (!passed) {
      failures++;
      Logger.recordOutput("Sim/Results/Failures", failures);
    }
  }

  boolean passed() {
    return report.passed();
  }

  /** Writes the report and prints a summary table. */
  void finish(Path reportFile, boolean stoppedEarly) {
    report.setStoppedEarly(stoppedEarly);
    try {
      ScenarioJson.writeRunReport(reportFile, report);
    } catch (IOException e) {
      System.err.println("[Sim] Could not write " + reportFile + ": " + e.getMessage());
    }
    StringBuilder sb = new StringBuilder("\n========== Simulation results ==========\n");
    for (String e : report.errors()) {
      sb.append("  ERROR  ").append(e).append('\n');
    }
    for (RunReport.Test t : report.tests()) {
      long passedChecks = t.checks().stream().filter(RunReport.Check::passed).count();
      sb.append(String.format(
          Locale.ROOT, "  %-4s  %-45s %-4s  %d/%d checks%n",
          t.passed() ? "PASS" : "FAIL", t.name(), t.alliance(), passedChecks, t.checks().size()));
      for (String e : t.errors()) {
        sb.append("          ! ").append(e).append('\n');
      }
      for (RunReport.Check c : t.checks()) {
        if (!c.passed()) {
          sb.append(String.format(Locale.ROOT, "          x %s @%.2fs: %s%n", c.label(), c.simTime(), c.actual()));
        }
      }
    }
    sb.append(stoppedEarly ? "  (stopped early)\n" : "")
        .append(report.passed() ? "  ALL PASSED" : "  FAILURES: " + failures)
        .append("\n  Report: ").append(reportFile).append('\n')
        .append("========================================\n");
    System.out.print(sb);
  }
}
