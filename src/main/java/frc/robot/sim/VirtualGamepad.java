package frc.robot.sim;

import frc.robot.sim.scenario.ScenarioEvent.ButtonAction;
import frc.robot.sim.scenario.ScenarioEvent.GamepadButton;
import frc.robot.sim.scenario.ScenarioEvent.PovDirection;
import frc.robot.sim.scenario.ScenarioEvent.TriggerSide;
import org.wpilib.driverstation.POVDirection;
import org.wpilib.driverstation.internal.DriverStationBackend;
import org.wpilib.simulation.DriverStationSim;

/**
 * A scripted Xbox controller. Holds button / POV / axis state and pushes it to {@link
 * DriverStationSim} once per loop with {@link #flush}.
 *
 * <p>Sticks use the same sign conventions as the robot code reads them: drive commands negate
 * {@code getLeftY()} / {@code getLeftX()} / {@code getRightX()}, so "forward" is LEFT_Y = -1.
 */
final class VirtualGamepad {
  // Axis indices on NiDsXboxController.
  private static final int LEFT_X = 0;
  private static final int LEFT_Y = 1;
  private static final int LEFT_TRIGGER = 2;
  private static final int RIGHT_TRIGGER = 3;
  private static final int RIGHT_X = 4;
  private static final int RIGHT_Y = 5;
  private static final int AXIS_COUNT = 6;
  /** GamepadButton is declared in NiDsXboxController.Button order, indices 0..9. */
  private static final int BUTTON_COUNT = GamepadButton.values().length;
  private static final int MAX_FLUSH_ATTEMPTS = 5;

  private final int port;
  private final double[] axes = new double[AXIS_COUNT];
  private int heldButtons;
  /** Buttons pressed for a single loop; cleared after the next flush. */
  private int momentaryButtons;
  private POVDirection pov = POVDirection.CENTER;
  private boolean momentaryPov;
  private String gameData = "";
  /** When false (manual mode), game data is left to the sim GUI. */
  private boolean manageGameData = true;
  private String name = "Simulator Xbox";

  VirtualGamepad(int port) {
    this.port = port;
  }

  void button(GamepadButton button, ButtonAction action) {
    int bit = 1 << button.ordinal();
    switch (action) {
      case PRESS -> {
        heldButtons |= bit;
        momentaryButtons |= bit;
      }
      case HOLD -> heldButtons |= bit;
      case RELEASE -> heldButtons &= ~bit;
    }
  }

  void pov(PovDirection direction, ButtonAction action) {
    pov = action == ButtonAction.RELEASE ? POVDirection.CENTER : POVDirection.valueOf(direction.name());
    momentaryPov = action == ButtonAction.PRESS;
  }

  void trigger(TriggerSide side, double value) {
    axes[side == TriggerSide.LEFT ? LEFT_TRIGGER : RIGHT_TRIGGER] = value;
  }

  /** Driver-relative translation: x forward, y left. */
  void drive(double x, double y) {
    axes[LEFT_Y] = -x;
    axes[LEFT_X] = -y;
  }

  /** Rotation, positive = counter-clockwise. */
  void turn(double ccwRate) {
    axes[RIGHT_X] = -ccwRate;
  }

  void stopSticks() {
    axes[LEFT_X] = 0;
    axes[LEFT_Y] = 0;
    axes[RIGHT_X] = 0;
    axes[RIGHT_Y] = 0;
  }

  void gameData(String message) {
    gameData = message == null ? "" : message;
  }

  /**
   * Replaces the whole controller state at once (used by manual mode to relay a real gamepad).
   *
   * @param newAxes 6 axes in NiDsXboxController order
   * @param buttons button bits in NiDsXboxController order
   * @param povBits HAL POV bits (0 = centred)
   */
  void setAll(double[] newAxes, int buttons, int povBits) {
    System.arraycopy(newAxes, 0, axes, 0, AXIS_COUNT);
    heldButtons = buttons & ((1 << BUTTON_COUNT) - 1);
    momentaryButtons = 0;
    pov = POVDirection.of((byte) povBits);
    momentaryPov = false;
  }

  /** Stop writing game data (manual mode: the sim GUI owns it). */
  void leaveGameDataAlone() {
    manageGameData = false;
  }

  /** Name the robot code sees for this controller. */
  void name(String newName) {
    name = newName;
  }

  /** Makes the port report no controller (zero inputs available). */
  void disconnect() {
    reset();
    DriverStationSim.setJoystickAxesAvailable(port, 0);
    DriverStationSim.setJoystickButtonsAvailable(port, 0);
    DriverStationSim.setJoystickPOVsAvailable(port, 0);
    DriverStationSim.setJoystickIsGamepad(port, false);
    DriverStationSim.setJoystickName(port, "");
    DriverStationSim.notifyNewData();
  }

  /** Releases every input and clears game data. */
  void reset() {
    java.util.Arrays.fill(axes, 0.0);
    heldButtons = 0;
    momentaryButtons = 0;
    pov = POVDirection.CENTER;
    momentaryPov = false;
    gameData = "";
  }

  /**
   * Pushes the current state to the simulated driver station, then releases one-loop presses.
   *
   * @return false if the DS still reports the wrong controller layout after several attempts
   */
  boolean flush() {
    boolean ok = false;
    for (int attempt = 1; attempt <= MAX_FLUSH_ATTEMPTS && !ok; attempt++) {
      describeController();
      for (int i = 0; i < AXIS_COUNT; i++) {
        DriverStationSim.setJoystickAxis(port, i, axes[i]);
      }
      if (manageGameData) {
        DriverStationSim.setGameData(gameData);
      }
      DriverStationSim.setJoystickPOV(port, 0, pov);
      for (int i = 0; i < BUTTON_COUNT; i++) {
        DriverStationSim.setJoystickButton(port, i, (heldButtons & (1 << i)) != 0);
      }
      // notifyNewData() occasionally (about 1 in 5000 calls) fails to apply; verify and retry.
      DriverStationSim.notifyNewData();
      ok = DriverStationBackend.getStickAxesMaximumIndex(port) == AXIS_COUNT
          && DriverStationBackend.getStickButtonsMaximumIndex(port) == BUTTON_COUNT;
      if (ok && attempt > 2) {
        System.out.println("[Sim] Warning: controller update needed " + attempt + " attempts");
      }
    }
    heldButtons &= ~momentaryButtons;
    momentaryButtons = 0;
    if (momentaryPov) {
      pov = POVDirection.CENTER;
      momentaryPov = false;
    }
    return ok;
  }

  /**
   * Tells the DS this port is an Xbox-style gamepad with 6 axes, 10 buttons and 1 POV.
   *
   * <p>In WPILib 2027 "available" is a bitmask (bit i = input i exists) and robot code reads 0 for
   * any input whose bit is clear, so use the *MaximumIndex setters, which build the mask from a
   * count. ({@code setJoystickAxesAvailable(port, 6)} would enable only axes 1 and 2.)
   */
  private void describeController() {
    DriverStationSim.setJoystickIsGamepad(port, true);
    DriverStationSim.setJoystickName(port, name);
    DriverStationSim.setJoystickAxesMaximumIndex(port, AXIS_COUNT);
    DriverStationSim.setJoystickButtonsMaximumIndex(port, BUTTON_COUNT);
    DriverStationSim.setJoystickPOVsMaximumIndex(port, 1);
  }
}
