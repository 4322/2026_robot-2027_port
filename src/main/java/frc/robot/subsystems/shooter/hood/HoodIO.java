package frc.robot.subsystems.shooter.hood;

import org.littletonrobotics.junction.AutoLog;

public interface HoodIO {
  @AutoLog
  public static class HoodIOInputs {
    public boolean motorConnected = false;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double tempCelsius = 0.0;
    public double hoodRPS = 0.0;
    public double hoodDegrees = 0.0;
  }

  public default void updateInputs(HoodIOInputs inputs) {}

  public default void setHomed() {}

  public default void setAngle(double angle) {}

  public default void setVoltage(double voltage) {}

  public default void setBrakeMode(boolean mode) {}
}
