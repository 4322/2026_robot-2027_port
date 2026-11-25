package frc.robot.util.ClockworkLib;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class GenericArmIOSim implements GenericMotorIO {
  private final SingleJointedArmSim sim;
  private double appliedVolts = 0.0;

  public GenericArmIOSim(DCMotor motor, double gearing, double jKgMetersSquared) {
    this.sim = new SingleJointedArmSim(motor, gearing, jKgMetersSquared, 0.5, -Math.PI, Math.PI, true, 0.0);
  }

  @Override
  public void updateInputs(GenericMotorIOInputs inputs) {
    sim.update(0.02);
    inputs.connected = true;
    inputs.appliedVolts = appliedVolts;
    inputs.velocityRPS = sim.getVelocityRadPerSec() / (2 * Math.PI);
    inputs.positionDeg = Math.toDegrees(sim.getAngleRads());
    inputs.statorCurrentAmps = sim.getCurrentDrawAmps();
    inputs.supplyCurrentAmps = sim.getCurrentDrawAmps();
    inputs.tempCelcius = 25.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setPosition(double deg) {}

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
