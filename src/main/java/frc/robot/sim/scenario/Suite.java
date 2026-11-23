package frc.robot.sim.scenario;

import java.util.ArrayList;
import java.util.List;

/**
 * A named list of runs, stored in {@code simulation/suites/<id>.json}. Replaces the old
 * {@code RegressTests} enum.
 */
public class Suite {
  /**
   * One line of a suite.
   *
   * @param scenario scenario file id (file name without .json); null when {@code allAutos}
   * @param allAutos run every non-characterization auto routine as an auto-only scenario
   * @param alliances alliances to run it on, in order
   * @param autoDuration auto length for {@code allAutos} runs (null = default 20 s)
   */
  public record Run(String scenario, boolean allAutos, List<SimAlliance> alliances, Double autoDuration) {
    public Run {
      alliances = List.copyOf(alliances);
    }

    public static Run of(String scenario, SimAlliance... alliances) {
      return new Run(scenario, false, List.of(alliances), null);
    }
  }

  private String name;
  private final List<Run> runs = new ArrayList<>();

  public Suite(String name) {
    this.name = name;
  }

  public String getName() {
    return name;
  }

  public void setName(String name) {
    this.name = name;
  }

  /** Live, mutable list of runs. */
  public List<Run> runs() {
    return runs;
  }
}
