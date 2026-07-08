package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

public interface RollersIO {
  @AutoLog
  public static class RollersIOInputs {
    public boolean leaderConnected = false;
    public double leaderStatorCurrentAmps = 0.0;
    public double leaderSupplyCurrentAmps = 0.0;
    public double leaderTempCelcius = 0.0;
    public double leaderRotationsPerSec = 0.0;
    public double leaderVolts = 0.0;

    public boolean followerConnected = false;
    public double followerStatorCurrentAmps = 0.0;
    public double followerSupplyCurrentAmps = 0.0;
    public double followerTempCelcius = 0.0;
    public double followerRotationsPerSec = 0.0;
    public double followerVolts = 0.0;
  }

  public default void updateInputs(RollersIOInputs inputs) {}

  public default void setVoltage(double voltage) {}

  public default void stopMotor() {}

  public default TalonFX getTalonFX() {
    return null;
  }
}
