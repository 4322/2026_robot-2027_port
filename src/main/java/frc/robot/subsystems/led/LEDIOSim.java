package frc.robot.subsystems.led;

import com.ctre.phoenix6.signals.RGBWColor;
import frc.robot.subsystems.led.LED.AnimationType;

public class LEDIOSim implements LEDIO {
  private String ledColor;

  public LEDIOSim() {}

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    inputs.ledsConnected = true;
    inputs.ledColor = ledColor;
  }

  @Override
  public void clearLEDs() {
    ledColor = "none";
  }

  @Override
  public void setLEDs(AnimationType animation, int slot, int r, int g, int b) {
    ledColor = animation.toString() + ": " + new RGBWColor(r, g, b).toString();
  }

  @Override
  public void setLEDs(AnimationType animation, int slot) {
    ledColor = animation.toString();
  }
}
