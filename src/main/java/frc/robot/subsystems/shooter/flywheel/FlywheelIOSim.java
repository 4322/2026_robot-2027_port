package frc.robot.subsystems.shooter.flywheel;

import frc.robot.constants.Constants;

public class FlywheelIOSim implements FlywheelIO {
  private double rate = 0.1;
  private double mechanismRotationsPerSec = 0.0;
  private double targetVelocity = 0.0;

  @Override
  public void updateInputs(FlywheelIOInputs inputs) {
    inputs.leaderMotorConnected = true;
    inputs.followerMotorConnected = true;

    simVelocity();
    inputs.leaderMechanismRPS = mechanismRotationsPerSec;
    inputs.followerMechanismRPS = mechanismRotationsPerSec;
  }

  @Override
  public void setTargetMechanismRPS(double velocity) {
    targetVelocity = velocity;
  }

  @Override
  public void stop() {
    targetVelocity = 0.0;
  }

  private void simVelocity() {
    if (mechanismRotationsPerSec < targetVelocity) {
      mechanismRotationsPerSec += Constants.Sim.flywheelRate;
    } else if (mechanismRotationsPerSec > targetVelocity) {
      mechanismRotationsPerSec -= Constants.Sim.flywheelRate;
    }
  }
}
