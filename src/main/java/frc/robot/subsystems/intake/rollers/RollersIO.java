package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public boolean Connected = false;
    public double StatorCurrentAmps = 0.0;
    public double SupplyCurrentAmps = 0.0;
    public double TempCelcius = 0.0;
    public double RotationsPerSec = 0.0;
    public double Volts = 0.0;
  }
  
  public default void updateInputs(RollersIOInputs inputs) {}

  public default void setVoltage(double volts) {}

  public default void stop() {}

  public default TalonFX getTalonFX() { return null; }
}
