package frc.robot.subsystems.shooter.spindexer;

import frc.robot.constants.Constants;

public class SpindexerIOSim implements SpindexerIO {
  private double mechanismRotationsPerSec = 0.0;
  private double targetVelocity = 0.0;
  private boolean isStalled = false;

  @Override
  public void updateInputs(SpindexerIOInputs inputs) {
    inputs.leaderMotorConnected = true;
    inputs.followerMotorConnected = true;
    simVelocity();
    inputs.leaderMechanismRPS = mechanismRotationsPerSec;
    inputs.followerMechanismRPS = mechanismRotationsPerSec;
  }

  @Override
  public void setTargetMechanismRotations(double velocity) {
    targetVelocity = velocity;
    
  }

  @Override 
  public void setTargetVoltage(double voltage) {
    
    targetVelocity = 0.75 * voltage;
  }

  @Override
  public void stop() {
    targetVelocity = 0.0;
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    return;
  }

  private void simVelocity() {
    if (mechanismRotationsPerSec < targetVelocity) {
      mechanismRotationsPerSec += Constants.Sim.spindexerRate;
    } else if (mechanismRotationsPerSec > targetVelocity) {
      mechanismRotationsPerSec -= Constants.Sim.spindexerRate;
    }
  }

  @Override 
  public void simStall(boolean stall) {
    isStalled = stall;
    if (stall) {
      mechanismRotationsPerSec = 0;
    }
  }
}
