package frc.robot.sim.scenario;

import java.util.ArrayList;
import java.util.List;

/**
 * The auto or teleop half of a scenario: a list of timed events plus segment settings.
 *
 * <p>Mutable so the editor GUI can change it in place.
 */
public class Segment {
  public static final double DEFAULT_AUTO_DURATION = 20.0;
  public static final double DEFAULT_TELEOP_MATCH_TIME = 140.0;

  /** True for the auto segment, false for teleop. */
  private final boolean auto;

  /** Auto only: the {@code AutoName} to run, e.g. "R_CODEORANGE". Null for teleop. */
  private String autoRoutine;

  /**
   * Minimum length of the segment in seconds. The segment ends at the later of this and the last
   * event. Auto defaults to {@link #DEFAULT_AUTO_DURATION}; teleop defaults to "last event".
   */
  private Double durationSec;

  /**
   * Match time (seconds remaining) at the start of the segment; counts down while it runs. Defaults
   * to the auto duration for auto, {@link #DEFAULT_TELEOP_MATCH_TIME} for teleop.
   */
  private Double startMatchTime;

  private final List<ScenarioEvent> events = new ArrayList<>();

  public Segment(boolean auto) {
    this.auto = auto;
  }

  public Segment(boolean auto, String autoRoutine, Double durationSec, Double startMatchTime, List<ScenarioEvent> events) {
    this.auto = auto;
    this.autoRoutine = autoRoutine;
    this.durationSec = durationSec;
    this.startMatchTime = startMatchTime;
    this.events.addAll(events);
  }

  public String getAutoRoutine() {
    return autoRoutine;
  }

  public void setAutoRoutine(String autoRoutine) {
    this.autoRoutine = autoRoutine;
  }

  public Double getDurationSec() {
    return durationSec;
  }

  public void setDurationSec(Double durationSec) {
    this.durationSec = durationSec;
  }

  public Double getStartMatchTime() {
    return startMatchTime;
  }

  public void setStartMatchTime(Double startMatchTime) {
    this.startMatchTime = startMatchTime;
  }

  /** Live, mutable list of events. */
  public List<ScenarioEvent> events() {
    return events;
  }

  public boolean isAuto() {
    return auto;
  }

  /** Absolute time (seconds from segment start) of each event. */
  public double[] absoluteTimes() {
    double[] times = new double[events.size()];
    double t = 0.0;
    for (int i = 0; i < events.size(); i++) {
      t += events.get(i).waitSec();
      times[i] = t;
    }
    return times;
  }

  /** Time of the last event, or 0 if there are none. */
  public double lastEventTime() {
    double[] times = absoluteTimes();
    return times.length == 0 ? 0.0 : times[times.length - 1];
  }

  /** When the segment ends: the later of the minimum duration and the last event. */
  public double endTime() {
    double minDuration = durationSec != null ? durationSec : (isAuto() ? DEFAULT_AUTO_DURATION : 0.0);
    return Math.max(minDuration, lastEventTime());
  }

  /** Match time remaining at the start of this segment. */
  public double effectiveStartMatchTime() {
    if (startMatchTime != null) {
      return startMatchTime;
    }
    return isAuto() ? endTime() : DEFAULT_TELEOP_MATCH_TIME;
  }

  /** Deep-ish copy (events are immutable records). */
  public Segment copy() {
    return new Segment(auto, autoRoutine, durationSec, startMatchTime, events);
  }
}
