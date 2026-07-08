package frc.robot.subsystems.shooter.turret;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.geometry.Pose3d;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.util.ClockUtil;
import org.littletonrobotics.junction.Logger;

public class Turret {
  private TurretIO io;
  private TurretIOInputsAutoLogged inputs = new TurretIOInputsAutoLogged();
  public boolean needsToUnwind = false;
  private Double desiredDeg = 0.0;
  private Double azimuth = 0.0;
  private double prevDeg = 0.0;
  private Timer atGoalTimer = new Timer();
  private boolean isScoring = false;
  private Timer crtTimer = new Timer();
  private double ffRadPerrSec = 0;
  private boolean unjamOverride;
  private double unjamDeg = 0.0;

  public enum turretState {
    DISABLED,
    SET_TURRET_ANGLE,
  }

  public turretState state = turretState.DISABLED;

  public Turret(TurretIO io) {
    this.io = io;
    // Temporary until we can,
    // Temporary until we can,
    io.updateInputs(inputs);
    io.setPosition(getRotation()); // adjust for offset locked (calibrated) position
    crtTimer.start();
  }

  public void inputsPeriodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Turret", inputs);
  }

  public void outputsPeriodic() {
    if (crtTimer.hasElapsed(2.0)) {
      crtTimer.restart();
      Logger.recordOutput("Shooter/Turret/crtDegrees", Units.rotationsToDegrees(getRotation()));
    }
    switch (Constants.turretMode) {
      case DISABLED -> {}
      case TUNING -> {}
      case NORMAL -> {
        updateAtGoalTimer();

        if (DriverStation.isDisabled()) {
          state = turretState.DISABLED;
        }

        switch (state) {
          case DISABLED -> {
            // If ever disabled during unwind, ensure variable is reset
            // Ensure turret remains in position upon re-enabling after moving while disabled
            desiredDeg = inputs.turretDegs;
            prevDeg = inputs.turretDegs;
            needsToUnwind = false;
            unjamOverride = false;
            if (!DriverStation.isDisabled()) {
              state = turretState.SET_TURRET_ANGLE;
            }
            break;
          }
          case SET_TURRET_ANGLE -> {
            // Doing it this way so we are
            // constantly updating new desired angle while unamming so we can immediately snap to
            // position and keep being offset if we move.
            if (unjamOverride) {
              io.setAngle(unjamDeg, 0);
            } else if (desiredDeg != null) {

              io.setAngle(desiredDeg, ffRadPerrSec);
            } else {

              io.setAngle(prevDeg, ffRadPerrSec);
            }
          }
        }

        updateNetworkTableValues();
        Logger.recordOutput("Shooter/Turret/State", state);
        Logger.recordOutput("Shooter/Turret/needToUnwind", needsToUnwind());
        Logger.recordOutput("Shooter/Turret/isUnwinding", isUnwinding());
        Logger.recordOutput("Shooter/Turret/atGoal", isAtGoal());
        Logger.recordOutput("Shooter/Turret/desiredDeg", desiredDeg);
        Logger.recordOutput("Shooter/Turret/unjamDeg", getTargetUnjamAngle());
        Logger.recordOutput(
            "Shooter/Turret/pose",
            new Pose3d[] {
              new Pose3d(
                  new Translation3d(
                      Constants.Turret.originToTurret.getX(),
                      Constants.Turret.originToTurret.getY(),
                      Units.inchesToMeters(10)),
                  new Rotation3d(0, 0, Units.degreesToRadians(getAngle() - 90)))
            });
      }
    }
  }

  public void requestAngle(Double angle, Boolean isScoring, double ffRadPerSec) {
    this.desiredDeg = angle;
    this.ffRadPerrSec = ffRadPerSec;
    azimuth = desiredDeg;
    if (Constants.turretLocked) {
      return;
    }

    if (state == turretState.SET_TURRET_ANGLE) {
      if (desiredDeg != null) {
        desiredDeg = calculateAngle(desiredDeg, inputs.turretDegs);
        if (needsToUnwind()) {
          desiredDeg =
              MathUtil.clamp(
                  desiredDeg,
                  Constants.Turret.minPhysicalLimitDeg,
                  Constants.Turret.maxPhysicalLimitDeg);
        }
        prevDeg = desiredDeg;
      } else {
        desiredDeg = prevDeg;
      }
    }
    this.isScoring = isScoring;
    updateAtGoalTimer();
  }

  private double getTargetAngleInMidpoint() {
    return (desiredDeg - Constants.Turret.midPointPhysicalDeg) > 0
        ? desiredDeg - 360
        : (desiredDeg - Constants.Turret.midPointPhysicalDeg) < 0 ? desiredDeg + 360 : desiredDeg;
  }

  private double getTargetUnjamAngle() {
    if (desiredDeg > inputs.turretDegs) {
      return MathUtil.clamp(
          inputs.turretDegs - Constants.Turret.unjamDeg,
          Constants.Turret.minUnwindLimitDeg,
          Constants.Turret.maxUnwindLimitDeg);
    } else if (desiredDeg < inputs.turretDegs) {
      return MathUtil.clamp(
          inputs.turretDegs + Constants.Turret.unjamDeg,
          Constants.Turret.minUnwindLimitDeg,
          Constants.Turret.maxUnwindLimitDeg);
    } else {
      return inputs.turretDegs;
    }
  }

  public boolean needsToUnwind() {
    return (inputs.turretDegs >= Constants.Turret.maxUnwindLimitDeg
        || inputs.turretDegs <= Constants.Turret.minUnwindLimitDeg);
  }

  public void unjamOverride(boolean override) {
    this.unjamOverride = override;
    if (override) {
      unjamDeg = getTargetUnjamAngle();
    }
  }

  public boolean isAtGoal() {
    if (Constants.turretLocked) {
      // desired turret angle is required robot heading when turret is locked
      return MathUtil.isNear(
          desiredDeg,
          RobotContainer.drive.getRotation().getDegrees(),
          Constants.Turret.goalToleranceLockedDeg);
    } else if (Constants.turretMode == Constants.SubsystemMode.DISABLED) {
      return true;
    } else {
      return Math.abs(inputs.turretDegs - desiredDeg) < Constants.Turret.smallToleranceDeg
          || (isScoring
              ? atGoalTimer.hasElapsed(Constants.scoringDoubleToleranceTime)
              : atGoalTimer.hasElapsed(Constants.passingDoubleToleranceTime));
    }
  }

  public void setTurretAngleState() {
    state = turretState.SET_TURRET_ANGLE;
  }

  public boolean atUnwindLimit() {
    return MathUtil.isNear(Constants.Turret.midPointPhysicalDeg, inputs.turretDegs, 180.1);
  }

  public boolean isUnwinding() {
    return this.needsToUnwind;
  }

  public void unwind(boolean needsUnwindFinish) {
    if (needsUnwindFinish) {
      if (needsToUnwind()) {
        desiredDeg = calculateAngle(azimuth, inputs.turretDegs);
      }
      desiredDeg =
          (MathUtil.isNear(Constants.Turret.midPointPhysicalDeg, desiredDeg, 180.1))
              ? desiredDeg
              : getTargetAngleInMidpoint();
    }
    this.needsToUnwind = needsUnwindFinish;
    prevDeg = desiredDeg;
    updateAtGoalTimer();
    Logger.recordOutput("Shooter/Turret/unwindDesiredDeg", prevDeg);
  }

  public double getAngle() {
    if (Constants.turretLocked) {
      return 90;
    } else {
      return inputs.turretDegs;
    }
  }

  public void setBrakeMode(Boolean mode) {
    io.setBrakeMode(mode);
  }

  private double calculateAngle(double targetAngle, double currentAngle) {
    // Sets minInclusive based on desired degree aka if +-180 % 180, the remainder is 0
    // So if it's 0 we don't know our desired angle, since it could +- 180
    // Then to fix this minInclusive is evaluated if our difference between our target angle
    // and current angle is +- 180 and go to it's respective angle
    boolean minInclusive;
    if (targetAngle - currentAngle == 180) {
      minInclusive = false;
    } else {
      minInclusive = true;
    }
    // The input modulus equation is used to find the distance to closest angle matching the target
    // angle based on
    // the current angle
    // and then add that distance of the current angle to set the target to the closest target
    // angle.
    targetAngle =
        ClockUtil.inputModulus(targetAngle - currentAngle, -180, 180, minInclusive) + currentAngle;

    return targetAngle;
  }

  private void updateAtGoalTimer() {
    if (Math.abs(inputs.turretDegs - desiredDeg) < Constants.Turret.largeToleranceDeg) {
      atGoalTimer.start();
    } else {
      atGoalTimer.stop();
      atGoalTimer.reset();
    }
  }

  public void requestTurretOverride(Double targetAngle) {
    desiredDeg =
        MathUtil.clamp(
            targetAngle, Constants.Turret.minUnwindLimitDeg, Constants.Turret.maxUnwindLimitDeg);
    this.ffRadPerrSec = 0;
    updateAtGoalTimer();
  }

  private static double crtToTurretRotations(double crtRotations) {
    /*
    Turret range is -180 degrees to +540 degrees
    Locked position/CRT zero is at +90 degrees
    490-760 degrees is -180 to +90 degrees
    0-450 degrees is +90 to +540 degrees

    Midpoint: +470 degrees; even though the turret can physically rotate more than
    2.111111 rotations, if we overrotate this gives us 20 degrees of buffer before
    we get into extreme trouble, although during normal operation we should never
    actually get into this range.

    (0, 0) should be -200 degrees. Our locked turret position is at +90 degrees, so
    that's +290 degrees of offset.

    290 degrees * 90/10 = encoder 1 should have rotated 7.25 rotations -> reads 0.25
    290 degrees * 90/19 = encoder 2 should have rotated 3.815789 rotations -> reads 3341.0/4096.0

    */

    return crtRotations - 200.0 / 360.0;
  }

  // Returns rotations relative to locked position
  // These rotations are in turret units
  private double getRotation() {

    // Based off of 4522's "brute force" solver:
    // https://www.chiefdelphi.com/uploads/short-url/vvrM1V1pqvDnnZfHtAhS02mBVIi.pdf

    // HACK: Look. I know this value works, okay?
    final int RATIO_LCM_SLOTS = 90;
    final double LCM = lcm(Constants.Turret.CANCoderOneRatio, Constants.Turret.CANCoderTwoRatio);
    // Range of motion in rotations
    final double ROTATIONAL_RANGE =
        LCM / Constants.Turret.CANCoderOneRatio / Constants.Turret.CANCoderTwoRatio;

    double encoderOne = inputs.encoderOneRot;
    double encoderTwo = inputs.encoderTwoRot;

    double bestError = Double.MAX_VALUE;
    double turretFullRotations = 0.5;

    // look for the entry in both lists of possible values that is the same.
    for (int i = 0; i < RATIO_LCM_SLOTS; i++) {
      // generate the candidate rotation value from encoder 1
      double candidate1 =
          mod((encoderOne + (double) i) / Constants.Turret.CANCoderOneRatio, ROTATIONAL_RANGE);
      for (int j = 0; j < RATIO_LCM_SLOTS; j++) {
        // generate the candidate rotation value from encoder 2
        double candidate2 =
            mod(((encoderTwo + (double) j) / Constants.Turret.CANCoderTwoRatio), ROTATIONAL_RANGE);
        double error = Math.abs(candidate2 - candidate1);
        if (error < bestError) {
          bestError = error;
          turretFullRotations = candidate1;
        }
      }
    }
    Logger.recordOutput("Shooter/Turret/rotationRange", ROTATIONAL_RANGE);
    Logger.recordOutput("Shooter/Turret/fullRotations", turretFullRotations);
    return crtToTurretRotations(turretFullRotations);
  }

  private void updateNetworkTableValues() {
    if (isAtGoal()) {
      if (Math.abs(inputs.turretDegs - desiredDeg) < Constants.Turret.smallToleranceDeg) {
        SmartDashboard.putString(
            "Turret/TurretAtGoal", Constants.NetworkTables.green.toHexString());
      } else {
        SmartDashboard.putString(
            "Turret/TurretAtGoal", Constants.NetworkTables.yellow.toHexString());
      }
    } else {
      SmartDashboard.putString("Turret/TurretAtGoal", Constants.NetworkTables.red.toHexString());
    }
  }

  private static double mod(double a, double b) {
    return ((a % b) + b) % b;
  }

  /**
   * Least common multiple that works on fractional values.
   *
   * @param a value a
   * @param b value b
   * @return least common multiple
   */
  private static double lcm(double a, double b) {
    double i = 1.0;
    while (true) {
      if (a * i % b < 0.001) {
        return a * i;
      }
      i += 1.0;
    }
  }
}
