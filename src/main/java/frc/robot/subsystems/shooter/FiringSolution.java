package frc.robot.subsystems.shooter;

public class FiringSolution {
  public double flywheelSpeedRPS;
  public double hoodAngle;
  public double turretAngleDeg;
  public double tunnelSpeedRPS;
  public double indexerSpeedRPS;

  public FiringSolution(
      double flywheelSpeedRPS,
      double hoodAngle,
      double turretAngleDeg,
      double tunnelSpeedRPS,
      double indexerSpeedRPS) {
    this.flywheelSpeedRPS = flywheelSpeedRPS;
    this.hoodAngle = hoodAngle;
    this.turretAngleDeg = turretAngleDeg;
    this.tunnelSpeedRPS = tunnelSpeedRPS;
    this.indexerSpeedRPS = indexerSpeedRPS;
  }
}
