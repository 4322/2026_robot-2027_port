package frc.robot.sim.scenario;

import java.util.ArrayList;
import java.util.List;

/**
 * Pass/fail results of a simulator run, written to {@code simulation/results/latest.json} and
 * shown in the editor GUI's Run tab.
 */
public class RunReport {
  /**
   * Result of one {@code expect} event.
   *
   * @param segment "auto" or "teleop"
   * @param eventIndex index of the expect event within its segment
   * @param label expect label (or generated description)
   * @param passed whether the check passed
   * @param actual last observed value, for diagnosing failures
   * @param simTime seconds from segment start when it resolved
   */
  public record Check(String segment, int eventIndex, String label, boolean passed, String actual, double simTime) {}

  /** Result of one scenario × alliance run. */
  public static class Test {
    private final String name;
    private final String scenarioId;
    private final SimAlliance alliance;
    private final List<Check> checks = new ArrayList<>();
    private final List<String> errors = new ArrayList<>();

    public Test(String name, String scenarioId, SimAlliance alliance) {
      this.name = name;
      this.scenarioId = scenarioId;
      this.alliance = alliance;
    }

    public String name() {
      return name;
    }

    /** Scenario file id, or null for generated (all-autos) scenarios. */
    public String scenarioId() {
      return scenarioId;
    }

    public SimAlliance alliance() {
      return alliance;
    }

    public List<Check> checks() {
      return checks;
    }

    /** Problems that aren't a specific check (auto not found, controller desync, ...). */
    public List<String> errors() {
      return errors;
    }

    public boolean passed() {
      return errors.isEmpty() && checks.stream().allMatch(Check::passed);
    }
  }

  private final List<Test> tests = new ArrayList<>();
  private final List<String> errors = new ArrayList<>();
  /** True when the run was stopped early (stop flag) rather than finishing every test. */
  private boolean stoppedEarly;

  public List<Test> tests() {
    return tests;
  }

  /** Run-level problems (bad run.json, missing scenario file, ...). */
  public List<String> errors() {
    return errors;
  }

  public boolean isStoppedEarly() {
    return stoppedEarly;
  }

  public void setStoppedEarly(boolean stoppedEarly) {
    this.stoppedEarly = stoppedEarly;
  }

  public boolean passed() {
    return errors.isEmpty() && tests.stream().allMatch(Test::passed);
  }
}
