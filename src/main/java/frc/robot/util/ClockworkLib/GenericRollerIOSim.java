package frc.robot.util.ClockworkLib;

import org.wpilib.simulation.DCMotorSim;
import org.wpilib.simulation.FlywheelSim;


public class GenericRollerIOSim implements GenericMotorIO {
  private final FlywheelSim sim;
  private double appliedVolts = 0.0;

  public GenericRollerIOSim(DCMotorSim motor, double gearing, double jKgMetersSquared) {
    this.sim = new FlywheelSim(motor, gearing, jKgMetersSquared);
  }

  @Override
  public void updateInputs(GenericMotorIOInputs inputs) {
    sim.update(0.02);
    inputs.connected = true;
    inputs.appliedVolts = appliedVolts;
    inputs.velocityRPS = sim.getAngularVelocity() / 60.0;
    inputs.positionDeg = 0.0;
    inputs.statorCurrentAmps = sim.getCurrentDraw();
    inputs.supplyCurrentAmps = sim.getCurrentDraw();
    inputs.tempCelcius = 25.0;
  }

  @Override
  public void setVoltage(double volts) {
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }
}
