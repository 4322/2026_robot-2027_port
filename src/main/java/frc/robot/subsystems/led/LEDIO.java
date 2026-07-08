package frc.robot.subsystems.led;

import frc.robot.subsystems.led.LED.AnimationType;

public interface LEDIO {
  public static class LEDIOInputs {
    public boolean ledsConnected = false;
    public String ledColor = "";
  }

  public default void updateInputs(LEDIOInputs inputs) {}

  public default void clearLEDs() {}

  public default void setLEDs(AnimationType animation, int slot, int r, int g, int b) {}

  public default void setLEDs(AnimationType animation, int slot) {}
}
