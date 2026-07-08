package frc.robot.subsystems.shooter.tunnel;

import org.littletonrobotics.junction.AutoLog;

public interface TunnelIO {

  @AutoLog
  public static class TunnelIOInputs {
    public boolean motorConnected = false;
    public double voltage = 0.0;
    public double mechanismRPS = 0.0;
    public double supplyAmps = 0.0;
    public double supplyAmpsAbsAvg = 0.0;
    public double statorAmps = 0.0;
    public double motorTempC = 0.0;
  }

  public default void updateInputs(TunnelIOInputs inputs) {}

  public default void setTargetMechanismRotations(double velocity) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}
}
