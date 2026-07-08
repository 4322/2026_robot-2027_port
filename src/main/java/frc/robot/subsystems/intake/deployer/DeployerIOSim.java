package frc.robot.subsystems.intake.deployer;

import org.wpilib.math.util.Units;
import org.wpilib.driverstation.DriverStation;
import frc.robot.constants.Constants;

public class DeployerIOSim implements DeployerIO {
  private double requestedVoltage = 0;
  private double voltage = 0;
  private double currentAngle =
      Constants.Deployer.maxGravityDegrees
          + Units.rotationsToDegrees(Constants.Deployer.SesnorOffsetRotations);
  private double requestedAngle = currentAngle;
  private double undefinedVoltage = -20;

  private double fastRate = 1;

  @Override
  public void updateInputs(DeployerIOInputs inputs) {
    inputs.motorConnected = true;

    double prevPos = currentAngle;
    simPos();
    simVolts();
    double velocity = Units.degreesToRotations(currentAngle - prevPos) * 50;
    inputs.angleDeg = currentAngle;
    inputs.appliedVolts = voltage;
    inputs.motorDegreesPerSec = velocity;
  }

  @Override
  public void setPosition(double requestedPosDeg) {
    this.requestedAngle = requestedPosDeg;
  }

  @Override
  public void stop() {}

  @Override
  public void seedPosition(double newAngleDeg) {
    currentAngle = newAngleDeg;
    this.requestedAngle = newAngleDeg;
  }

  private void simVolts() {
    if (DriverStation.isEnabled()) {
      if (requestedVoltage == undefinedVoltage) {
        voltage = 0;
      } else if (voltage < requestedVoltage) {
        voltage += (requestedVoltage - voltage) * fastRate;
      } else {
        voltage -= (voltage - requestedVoltage) * fastRate;
      }
    }
  }

  private void simPos() {
    if (DriverStation.isEnabled()) {
      if (currentAngle < requestedAngle) {
        currentAngle += (requestedAngle - currentAngle) * fastRate;
      } else {
        currentAngle -= (currentAngle - requestedAngle) * fastRate;
      }
    }
  }
}
