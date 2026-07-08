package frc.robot.subsystems.shooter.hood;

import org.wpilib.driverstation.DriverStation;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Hood {
  private static final LoggedTunableNumber largeToleranceDeg =
      new LoggedTunableNumber("Hood/largeToleranceDeg", Constants.Hood.largeToleranceDeg);
  private static final LoggedTunableNumber smallToleranceDeg =
      new LoggedTunableNumber("Hood/smallToleranceDeg", Constants.Hood.smallToleranceDeg);

  private HoodIO io;
  private HoodIOInputsAutoLogged inputs = new HoodIOInputsAutoLogged();
  private double requestedAngleDeg;
  private Timer atGoalTimer = new Timer();
  private Timer homingTimer = new Timer();
  private boolean homed = false;
  private boolean trenchOverride = false;
  private boolean isScoring = false;

  public Hood(HoodIO io) {
    this.io = io;
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Hood", inputs);
  }

  public void outputsPeriodic() {
    updateAtGoalTimer();
    switch (Constants.hoodMode) {
      case DISABLED -> {
        homed = true;
      }
      case TUNING -> {}
      case NORMAL -> {
        if (Constants.currentMode == Constants.Mode.SIM && !homed) {
          homed = true;
        }
        if (!homed) {
          homeHood();
        } else {
          io.setAngle(requestedAngleDeg);
          updateAtGoalTimer();
        }
        updateNetworkTableValues();
      }
    }
    Logger.recordOutput("Shooter/Hood/Timer", homingTimer.get());
    Logger.recordOutput("Shooter/Hood/homed", homed);
    Logger.recordOutput("Shooter/Hood/isAtGoal", isAtGoal());
    Logger.recordOutput("Shooter/Hood/goalDegrees", requestedAngleDeg);
  }

  private void homeHood() {
    if (!DriverStation.isEnabled()) {
      homingTimer.stop();
      homingTimer.reset();
      io.setVoltage(0);
    } else {
      io.setVoltage(Constants.Hood.homingVoltage);
      homingTimer.start();
      if (homingTimer.hasElapsed(Constants.Hood.minHomingSec)
          && Math.abs(inputs.hoodRPS) <= Constants.Hood.homingVelocityThresholdRPS) {
        io.setHomed();
        setGoal(Constants.Hood.safeAngleDeg);
        homed = true;
        homingTimer.stop();
        homingTimer.reset();
      }
    }
  }

  private void updateAtGoalTimer() {
    if (Math.abs(inputs.hoodDegrees - requestedAngleDeg) < largeToleranceDeg.get()) {
      atGoalTimer.start();
    } else {
      atGoalTimer.stop();
      atGoalTimer.reset();
    }
  }

  private void updateNetworkTableValues() {
    if (isAtGoal()) {
      if (isWithinSmallTolerance()) {
        SmartDashboard.putString("Hood/HoodAtGoal", Constants.NetworkTables.green.toHexString());
      } else {
        SmartDashboard.putString("Hood/HoodAtGoal", Constants.NetworkTables.yellow.toHexString());
      }
    } else {
      SmartDashboard.putString("Hood/HoodAtGoal", Constants.NetworkTables.red.toHexString());
    }
  }

  public void requestGoal(double degrees, boolean isScoring) {
    this.isScoring = isScoring;
    if (Constants.hoodMode == SubsystemMode.NORMAL && !trenchOverride) {
      setGoal(degrees);
    }
  }

  private void setGoal(double degrees) {
    requestedAngleDeg = degrees;
    updateAtGoalTimer();
  }

  public void trenchOverride(boolean override) {
    setGoal(Constants.Hood.safeAngleDeg);
    trenchOverride = override;
  }

  public boolean isSafeAngle() {
    return inputs.hoodDegrees < Constants.Hood.safeAngleDeg + Constants.Hood.largeToleranceDeg;
  }

  public void rehome() {
    homed = false;
  }

  public boolean isAtGoal() {
    if (!homed || trenchOverride) {
      return false;
    } else if (Constants.currentMode == Constants.Mode.SIM) {
      return true; // TODO temporary until we get hood sim working
    } else if (isWithinSmallTolerance()
        || (isScoring
            ? atGoalTimer.hasElapsed(Constants.scoringDoubleToleranceTime)
            : atGoalTimer.hasElapsed(Constants.passingDoubleToleranceTime))) {
      return true;
    } else {
      return false;
    }
  }

  private boolean isWithinSmallTolerance() {
    return Math.abs(inputs.hoodDegrees - requestedAngleDeg) < smallToleranceDeg.get();
  }

  public boolean isHomed() {
    return homed;
  }

  public double getPositionDegrees() {
    return inputs.hoodDegrees;
  }

  public void setBrakeMode(boolean brake) {
    io.setBrakeMode(brake);
  }
}
