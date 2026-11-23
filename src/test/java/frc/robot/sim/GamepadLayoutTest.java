package frc.robot.sim;

import static org.junit.jupiter.api.Assertions.assertArrayEquals;
import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

class GamepadLayoutTest {
  // 2027 Gamepad indices
  private static final int GP_LEFT_BUMPER = 9;
  private static final int GP_RIGHT_BUMPER = 10;
  private static final int GP_BACK = 4;
  private static final int GP_START = 6;
  private static final int GP_LEFT_STICK = 7;
  private static final int GP_RIGHT_STICK = 8;

  @Test
  void axesMoveToXboxSlots() {
    // 2027 order: LX, LY, RX, RY, LT, RT
    double[] in = {0.1, 0.2, 0.3, 0.4, 0.5, 0.6};
    // NI DS order: LX, LY, LT, RT, RX, RY
    assertArrayEquals(new double[] {0.1, 0.2, 0.5, 0.6, 0.3, 0.4}, GamepadLayout.fromGamepad(in, 0).axes(), 1e-12);
  }

  @Test
  void leftTriggerNoLongerTurns() {
    double[] in = {0, 0, 0, 0, 1.0, 0}; // left trigger fully pressed
    double[] out = GamepadLayout.fromGamepad(in, 0).axes();
    assertEquals(1.0, out[2], 1e-12); // NI DS left trigger
    assertEquals(0.0, out[4], 1e-12); // NI DS right stick X (rotation) untouched
  }

  @Test
  void buttonsMoveToXboxSlots() {
    assertEquals(0b1111, GamepadLayout.fromGamepad(new double[6], 0b1111).buttons()); // A B X Y unchanged
    assertEquals(1 << 4, GamepadLayout.fromGamepad(new double[6], 1L << GP_LEFT_BUMPER).buttons());
    assertEquals(1 << 5, GamepadLayout.fromGamepad(new double[6], 1L << GP_RIGHT_BUMPER).buttons());
    assertEquals(1 << 6, GamepadLayout.fromGamepad(new double[6], 1L << GP_BACK).buttons());
    assertEquals(1 << 7, GamepadLayout.fromGamepad(new double[6], 1L << GP_START).buttons());
    assertEquals(1 << 8, GamepadLayout.fromGamepad(new double[6], 1L << GP_LEFT_STICK).buttons());
    assertEquals(1 << 9, GamepadLayout.fromGamepad(new double[6], 1L << GP_RIGHT_STICK).buttons());
    assertEquals(0, GamepadLayout.fromGamepad(new double[6], 1L << 5).buttons()); // guide has no Xbox slot
  }

  @Test
  void dpadBecomesPov() {
    assertEquals(GamepadLayout.POV_UP, GamepadLayout.fromGamepad(new double[6], 1L << 11).pov());
    assertEquals(GamepadLayout.POV_DOWN | GamepadLayout.POV_LEFT,
        GamepadLayout.fromGamepad(new double[6], (1L << 12) | (1L << 13)).pov());
    assertEquals(0, GamepadLayout.fromGamepad(new double[6], (1L << 11) | (1L << 12)).pov()); // up+down cancel
    assertEquals(0, GamepadLayout.fromGamepad(new double[6], 1L << 11).buttons()); // d-pad isn't a button
  }

  @Test
  void missingAxesReadZero() {
    assertArrayEquals(new double[] {0.5, 0, 0, 0, 0, 0}, GamepadLayout.fromGamepad(new double[] {0.5}, 0).axes(), 1e-12);
  }
}
