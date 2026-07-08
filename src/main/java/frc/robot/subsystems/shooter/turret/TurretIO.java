package frc.robot.subsystems.shooter.turret;

import org.littletonrobotics.junction.AutoLog;

public interface TurretIO {
  @AutoLog
  public static class TurretIOInputs {
    public boolean motorConnected = false;
    public boolean encoderOneConnected = false;
    public boolean encoderTwoConnected = false;
    public double motorRPS = 0.0;
    public double supplyCurrentAmps = 0.0;
    public double statorCurrentAmps = 0.0;
    public double appliedVoltage = 0.0;
    public double tempCelsius = 0.0;
    public double encoderOneRot = 0.0;
    public double encoderTwoRot = 0.0;
    public double turretDegs = 0.0;
  }

  public default void updateInputs(TurretIOInputs inputs) {}

  public default void setAngle(double angle, double ffRadPerSec) {}

  public default void setBrakeMode(boolean mode) {}

  public default void setPosition(double rot) {}
}
