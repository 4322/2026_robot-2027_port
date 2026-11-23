package frc.robot.sim;

/**
 * Converts a controller in the WPILib 2027 {@code Gamepad} layout (what the sim GUI produces with
 * "Map gamepad" ticked) into the old NI Driver Station Xbox layout that the robot code reads through
 * {@code NiDsXboxController}.
 *
 * <pre>
 *                      2027 Gamepad      NI DS Xbox (robot code)
 *   left stick X / Y   axes 0 / 1        axes 0 / 1
 *   right stick X / Y  axes 2 / 3        axes 4 / 5
 *   left / right trig  axes 4 / 5        axes 2 / 3
 *   A B X Y            buttons 0-3       buttons 0-3
 *   back / start       buttons 4 / 6     buttons 6 / 7
 *   stick clicks L / R buttons 7 / 8     buttons 8 / 9
 *   bumpers L / R      buttons 9 / 10    buttons 4 / 5
 *   d-pad U D L R      buttons 11-14     POV 0
 * </pre>
 *
 * Plain Java (no WPILib calls) so it can be unit tested.
 */
final class GamepadLayout {
  private GamepadLayout() {}

  /** HAL POV bits. */
  static final int POV_UP = 0x01;
  static final int POV_RIGHT = 0x02;
  static final int POV_DOWN = 0x04;
  static final int POV_LEFT = 0x08;

  /** Old-layout controller state: 6 axes, 10 button bits, POV as HAL bits. */
  record XboxState(double[] axes, int buttons, int pov) {}

  /** NI DS axis index → 2027 Gamepad axis index. */
  private static final int[] AXIS_FROM = {0, 1, 4, 5, 2, 3};

  /** NI DS button index → 2027 Gamepad button index. */
  private static final int[] BUTTON_FROM = {0, 1, 2, 3, 9, 10, 4, 6, 7, 8};

  private static final int DPAD_UP = 11;
  private static final int DPAD_DOWN = 12;
  private static final int DPAD_LEFT = 13;
  private static final int DPAD_RIGHT = 14;

  /**
   * @param gamepadAxes axis values in 2027 Gamepad order (missing axes read as 0)
   * @param gamepadButtons button bits in 2027 Gamepad order
   */
  static XboxState fromGamepad(double[] gamepadAxes, long gamepadButtons) {
    double[] axes = new double[AXIS_FROM.length];
    for (int i = 0; i < axes.length; i++) {
      int from = AXIS_FROM[i];
      axes[i] = from < gamepadAxes.length ? gamepadAxes[from] : 0.0;
    }
    int buttons = 0;
    for (int i = 0; i < BUTTON_FROM.length; i++) {
      if (bit(gamepadButtons, BUTTON_FROM[i])) {
        buttons |= 1 << i;
      }
    }
    int pov = 0;
    if (bit(gamepadButtons, DPAD_UP)) {
      pov |= POV_UP;
    }
    if (bit(gamepadButtons, DPAD_RIGHT)) {
      pov |= POV_RIGHT;
    }
    if (bit(gamepadButtons, DPAD_DOWN)) {
      pov |= POV_DOWN;
    }
    if (bit(gamepadButtons, DPAD_LEFT)) {
      pov |= POV_LEFT;
    }
    // opposite directions held together cancel out
    if ((pov & (POV_UP | POV_DOWN)) == (POV_UP | POV_DOWN)) {
      pov &= ~(POV_UP | POV_DOWN);
    }
    if ((pov & (POV_LEFT | POV_RIGHT)) == (POV_LEFT | POV_RIGHT)) {
      pov &= ~(POV_LEFT | POV_RIGHT);
    }
    return new XboxState(axes, buttons, pov);
  }

  private static boolean bit(long bits, int index) {
    return (bits & (1L << index)) != 0;
  }
}
