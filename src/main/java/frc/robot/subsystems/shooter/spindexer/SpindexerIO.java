package frc.robot.subsystems.shooter.spindexer;

import org.littletonrobotics.junction.AutoLog;

public interface SpindexerIO {

  @AutoLog
  public static class SpindexerIOInputs {
    public boolean leaderMotorConnected = false;
    public double leaderVoltage = 0.0;
    public double leaderMechanismRPS = 0.0;
    public double leaderSupplyAmps = 0.0;
    public double leaderStatorAmps = 0.0;
    public double leaderMotorTempC = 0.0;

    public boolean followerMotorConnected = false;
    public double followerVoltage = 0.0;
    public double followerMechanismRPS = 0.0;
    public double followerSupplyAmps = 0.0;
    public double followerStatorAmps = 0.0;
    public double followerMotorTempC = 0.0;

    public double supplyAmpsAvg = 0.0;
  }

  public default void updateInputs(SpindexerIOInputs inputs) {}

  public default void setTargetMechanismRotations(double velocity) {}

  public default void stop() {}

  public default void enableBrakeMode(boolean enable) {}
}
