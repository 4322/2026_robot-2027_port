package frc.robot.subsystems.led;

import com.ctre.phoenix6.configs.CANdleConfiguration;
import com.ctre.phoenix6.controls.ColorFlowAnimation;
import com.ctre.phoenix6.controls.EmptyAnimation;
import com.ctre.phoenix6.controls.FireAnimation;
import com.ctre.phoenix6.controls.LarsonAnimation;
import com.ctre.phoenix6.controls.RainbowAnimation;
import com.ctre.phoenix6.controls.RgbFadeAnimation;
import com.ctre.phoenix6.controls.SingleFadeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import com.ctre.phoenix6.signals.RGBWColor;
import com.ctre.phoenix6.signals.StatusLedWhenActiveValue;
import frc.robot.constants.Constants;
import frc.robot.subsystems.led.LED.AnimationType;

public class LEDIOCANdle implements LEDIO {
  private CANdle leds;
  private CANdleConfiguration config;
  private String ledColor;

  public LEDIOCANdle() {
    leds = new CANdle(Constants.LED.CANdleID);
    config = new CANdleConfiguration();
    config.LED.StripType = Constants.LED.stripType;
    config.LED.BrightnessScalar = Constants.LED.brightnessScalar;
    config.CANdleFeatures.StatusLedWhenActive = StatusLedWhenActiveValue.Enabled;

    leds.getConfigurator().apply(config);
  }

  @Override
  public void updateInputs(LEDIOInputs inputs) {
    inputs.ledsConnected = leds.isConnected();
    inputs.ledColor = ledColor;
  }

  @Override
  public void clearLEDs() {
    ledColor = "none";
    for (int i = 0; i < 8; i++) {
      leds.setControl(new EmptyAnimation(i));
    }
  }

  @Override
  public void setLEDs(AnimationType animation, int slot, int r, int g, int b) {
    ledColor = animation.toString() + ": " + new RGBWColor(r, g, b).toString();
    switch (animation) {
      case COLOR_FLOW -> {
        leds.setControl(
            new ColorFlowAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
      case LARSON -> {
        leds.setControl(
            new LarsonAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
      case SINGLE_FADE -> {
        leds.setControl(
            new SingleFadeAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
      case SOLID_COLOR -> {
        leds.setControl(
            new SingleFadeAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
      case STROBE -> {
        leds.setControl(
            new SingleFadeAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
      case TWINKLE -> {
        leds.setControl(
            new SingleFadeAnimation(Constants.LED.ledStart, Constants.LED.ledEnd)
                .withSlot(slot)
                .withColor(new RGBWColor(r, g, b)));
      }
      case RAINBOW, FIRE, RGB_FADE -> {}
    }
  }

  @Override
  public void setLEDs(AnimationType animation, int slot) {
    ledColor = animation.toString();
    switch (animation) {
      case RAINBOW -> {
        leds.setControl(
            new RainbowAnimation(Constants.LED.ledStart, Constants.LED.ledEnd).withSlot(slot));
      }
      case RGB_FADE -> {
        leds.setControl(
            new RgbFadeAnimation(Constants.LED.ledStart, Constants.LED.ledEnd).withSlot(slot));
      }
      case FIRE -> {
        leds.setControl(
            new FireAnimation(Constants.LED.ledStart, Constants.LED.ledEnd).withSlot(slot));
      }
      case LARSON, SINGLE_FADE, SOLID_COLOR, STROBE, TWINKLE, COLOR_FLOW -> {}
    }
  }
}
