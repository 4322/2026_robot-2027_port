package frc.robot.subsystems.shooter.turret;

import org.wpilib.math.util.Units;

public class TurretIOSim implements TurretIO {
  private double requestedVoltage = 0;
  private double requestedAngle = Units.rotationsToDegrees(0.5);

  private double voltage = 0;
  private double currentAngle = Units.rotationsToDegrees(0.5);
  private double undefinedVoltage = -20;

  private double fastRate = 1.0;

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    inputs.motorConnected = true;

    double prevPos = currentAngle;
    simPos();
    simVolts();
    double velocity = (currentAngle - prevPos) * 50;

    inputs.turretDegs = currentAngle;
    inputs.appliedVoltage = voltage;
    inputs.motorRPS = velocity;
  }

  private void simVolts() {
    if (requestedVoltage == undefinedVoltage) {
      voltage = 0;
    } else if (voltage < requestedVoltage) {
      voltage += (requestedVoltage - voltage) * fastRate;
    } else {
      voltage -= (voltage - requestedVoltage) * fastRate;
    }
  }

  private void simPos() {
    if (currentAngle < requestedAngle) {
      currentAngle += (requestedAngle - currentAngle) * fastRate;
    } else {
      currentAngle -= (currentAngle - requestedAngle) * fastRate;
    }
  }

  @Override
  public void setAngle(double angleDeg, double ffRadPerSec) {
    this.requestedAngle = angleDeg;
  }

  @Override
  public void setPosition(double position) {
    this.currentAngle = Units.rotationsToDegrees(position);
  }
}
