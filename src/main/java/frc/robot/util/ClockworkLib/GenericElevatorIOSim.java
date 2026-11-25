package frc.robot.util.ClockworkLib;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class GenericElevatorIOSim implements GenericMotorIO {
  private final ElevatorSim sim;
  private double appliedVolts = 0.0;

  public GenericElevatorIOSim(DCMotor motor, double gearing, double massKg, double radiusMeters) {
    this.sim = new ElevatorSim(motor, gearing, massKg, radiusMeters, 0.0, 2.0, true, 0.0);
  }

  @Override
  public void updateInputs(GenericMotorIOInputs inputs) {
    sim.update(0.02);
    inputs.connected = true;
    inputs.appliedVolts = appliedVolts;
    inputs.velocityRPS = sim.getVelocityMetersPerSecond();
    inputs.positionDeg = sim.getPositionMeters();
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
  public void stop() {
    setVoltage(0.0);
  }
}
