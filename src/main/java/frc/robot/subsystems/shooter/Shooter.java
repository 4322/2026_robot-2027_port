package frc.robot.subsystems.shooter;

import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.RobotState;
import org.wpilib.system.Timer;
import org.wpilib.command2.SubsystemBase;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.constants.Constants.ShotCalculatorParameters;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.firingManager.FiringManager;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.tunnel.Tunnel;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPose;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.firecontrol.ShotCalculator;
import frc.robot.util.firecontrol.ShotLUT;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {

  public enum ShooterState {
    STARTING_CONFIG,
    DISABLED,
    IDLE, // Spindexer stopped, flywheel at full speed, tunnel at full speed
    UNWIND,
    PRESHOOT, // Flywheel gets up to speed; Turret/hood aim
    SHOOT, // Spindexer and tunnel get up to speed
    STOP // Everything but flywheel stopped
  }

  private ShooterState state = ShooterState.STARTING_CONFIG;
  private ShooterState requestedState = ShooterState.STARTING_CONFIG;

  private static final LoggedTunableNumber tunableFlywheelSpeedRPS =
      new LoggedTunableNumber("FiringManager/flywheelSpeedRPS", 0.0);
  private static final LoggedTunableNumber tunableHoodAngle =
      new LoggedTunableNumber("FiringManager/hoodAngle", 0.0);
  private static final LoggedTunableNumber tunableTunnelSpeedRPS =
      new LoggedTunableNumber("FiringManager/tunnelSpeedRPS", 0.0);
  private static final LoggedTunableNumber tunableIndexerSpeedRPS =
      new LoggedTunableNumber("FiringManager/indexerSpeedRPS", 0.0);

  private Flywheel flywheel;
  private Hood hood;
  private Spindexer spindexer;
  private Tunnel tunnel;
  private Turret turret;
  private Drive drive;
  private Double hoodOverrideDeg = null;
  private Double flywheelOverrideRPS = null;
  private Double turretOverrideDeg = null;

  private double targetHoodAngleDeg;
  private double targetFlywheelSpeedRPS;
  private double targetTurretAngleDeg;
  private double targetTunnelSpeedRPS;
  private double targetSpindexerSpeedRPS;
  private double targetFFRadPerSec;
  private boolean doUnwind = false;
  private boolean fixedPositionShooting = false;
  private boolean isScoring = true;
  private Timer idleTimer = new Timer();
  private boolean resetIdleTimeout = false;
  private boolean hasBeenScoring = false;
  private boolean isShooting = false;
  private Timer shootingTimeoutTimer = new Timer();
  private Timer simShootingTimer = new Timer();

  private ShotCalculator scoreCalc;
  private ShotCalculator passCalc;

  public Shooter(
      Flywheel flywheel,
      Hood hood,
      Spindexer spindexer,
      Tunnel tunnel,
      Turret turret,
      VisionGlobalPose visionGlobalPose,
      Drive drive) {
    this.flywheel = flywheel;
    this.hood = hood;
    this.spindexer = spindexer;
    this.tunnel = tunnel;
    this.turret = turret;
    this.drive = drive;

    ShotCalculator.Config config = new ShotCalculator.Config();
    config.launcherOffsetX =
        Constants.Turret.originToTurret
            .getX(); // how far forward the launcher is from robot center (m)
    config.launcherOffsetY = Constants.Turret.originToTurret.getY(); // how far left, 0 if centered
    config.phaseDelayMs = Constants.ShotCalculator.phaseDelayMs; // your vision pipeline latency
    config.mechLatencyMs =
        Constants.ShotCalculator.mechLatencyMs; // how long the mechanism takes to respond
    config.maxTiltDeg =
        Constants.ShotCalculator
            .maxTiltDeg; // suppress firing when chassis tilts past this (bumps/ramps)
    config.headingSpeedScalar =
        Constants.ShotCalculator
            .headingSpeedScalar; // heading tolerance tightens with robot speed (0 to disable)
    config.headingReferenceDistance =
        Constants.ShotCalculator
            .headingReferenceDistance; // heading tolerance scales with distance from hub
    config.minScoringDistance = 0;
    config.maxScoringDistance = 100;
    config.minSOTMSpeed = 0.1;
    config.maxSOTMSpeed = 100;
    config.tofMin = 0.05;
    config.tofMax = 100;

    scoreCalc = new ShotCalculator(config, true);
    passCalc = new ShotCalculator(config, false);

    ShotLUT scoreLUT = new ShotLUT();
    for (ShotCalculatorParameters params : Constants.FiringManager.firingParametersListScoring) {
      scoreLUT.put(
          params.distanceMeters(),
          params.flywheelRPS() * 60.0,
          params.hoodAngleDeg(),
          params.timeOfFlightSec());
    }
    scoreCalc.loadShotLUT(scoreLUT);

    ShotLUT passLUT = new ShotLUT();
    for (ShotCalculatorParameters params : Constants.FiringManager.firingParametersListPassing) {
      passLUT.put(
          params.distanceMeters(),
          params.flywheelRPS() * 60.0,
          params.hoodAngleDeg(),
          params.timeOfFlightSec());
    }
    passCalc.loadShotLUT(passLUT);
    shootingTimeoutTimer.stop();
    shootingTimeoutTimer.reset();
  }

  public double getTargetTurretAngleDeg() {
    return targetTurretAngleDeg;
  }

  @Override
  public void periodic() {
    flywheel.inputsPeriodic();
    hood.inputsPeriodic();
    tunnel.inputsPeriodic();
    spindexer.inputsPeriodic();
    if (!Constants.turretLocked) {
      turret.inputsPeriodic();
    }
  }

  public void outputsPeriodic() {
    calculateFiringSolution();

    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {

      flywheel.requestGoal(targetFlywheelSpeedRPS, false);
      hood.requestGoal(targetHoodAngleDeg, false);

      turret.requestAngle(targetTurretAngleDeg, false, 0);

      tunnel.requestGoal(targetTunnelSpeedRPS);
      spindexer.requestGoal(targetSpindexerSpeedRPS);

      flywheel.outputsPeriodic();
      hood.outputsPeriodic();
      tunnel.outputsPeriodic();
      spindexer.outputsPeriodic();

      if (!Constants.turretLocked) {
        turret.outputsPeriodic();
      }
      return;
    }
    if (RobotState.isDisabled() && state != ShooterState.STARTING_CONFIG) {
      // Reset variable in case disabling during unwind
      doUnwind = false;
      hoodOverrideDeg = null;
      flywheelOverrideRPS = null;
      turretOverrideDeg = null;
      state = ShooterState.DISABLED;
    }

    // Turret unwind logic blocks all other incoming state requests from commands
    // Once unwind is complete, other state requests are allowed and in cases where
    // no state request is made, goes back to previous state
    if (turret.needsToUnwind() || turret.isUnwinding()) {
      state = ShooterState.UNWIND;
    }

    switch (state) {
      case STARTING_CONFIG -> {
        // Intake has to be down first before using shooter
        // Commands will take state out of starting config only if intake was first deployed
      }
      case DISABLED -> {
        resetIdleTimeout = true;
        if (RobotState.isEnabled()) {
          state = ShooterState.IDLE;
        }
      }
      case IDLE -> {
        if (resetIdleTimeout) {
          idleTimer.restart();
          resetIdleTimeout = false;
        }

        spindexer.requestIdle();
        if (spindexer.isStopped()) {
          tunnel.requestIdle();
        }

        if (turretOverrideDeg != null) {
          turret.requestTurretOverride(turretOverrideDeg);
        } else {
          turret.requestAngle(targetTurretAngleDeg, isScoring, 0);
        }

        if (flywheelOverrideRPS != null) {
          flywheel.requestGoal(flywheelOverrideRPS, isScoring);
        } else if (idleTimer.hasElapsed(Constants.Flywheel.idleTimeout)) {
          flywheel.requestGoal(Constants.Flywheel.idleRPS, isScoring);
        } else {
          flywheel.requestGoal(targetFlywheelSpeedRPS, isScoring);
        }

        if (hoodOverrideDeg != null) {
          hood.requestGoal(hoodOverrideDeg, isScoring);
        } else if (idleTimer.hasElapsed(Constants.Hood.idleTimeout)) {

          hood.requestGoal(Constants.Hood.safeAngleDeg, isScoring);
        } else {
          hood.requestGoal(targetHoodAngleDeg, isScoring);
        }
      }

      case STOP -> {
        resetIdleTimeout = true;
        flywheel.requestGoal(0, isScoring);
        hood.requestGoal(targetHoodAngleDeg, isScoring);
        turret.requestAngle(targetTurretAngleDeg, isScoring, 0);
        spindexer.requestIdle();
        tunnel.requestIdle();
      }
      case UNWIND -> {
        // Optimization: Keep requesting target states while waiting for unwind if shooting
        if (requestedState == ShooterState.PRESHOOT) {
          hood.requestGoal(targetHoodAngleDeg, isScoring);
          flywheel.requestGoal(targetFlywheelSpeedRPS, isScoring);
        }

        // Keep sending turret angle requests and unwind logic will continuously adjust target
        // setpoint within range of physical midpoint
        turret.requestAngle(targetTurretAngleDeg, isScoring, 0);
        spindexer.requestIdle();
        tunnel.requestIdle();

        if (spindexer.isStopped()) {
          // Since requested turret angle is constantly updating while we're unwinding
          // turret will attempt to go to requested setpoint within certain range of physical
          // midpoint
          doUnwind = true;
        }

        if (turret.atUnwindLimit() || Constants.turretLocked) {
          doUnwind = false;
          // Exit unwind state when completed
          // In next loop cycle if no command sends shooter request,
          // shooter will go to last requested state before unwind started
          state = requestedState;
        }
        turret.unwind(doUnwind);
      }
      case PRESHOOT -> {
        resetIdleTimeout = true;
        spindexer.requestIdle();
        flywheel.requestGoal(targetFlywheelSpeedRPS, isScoring);
        hood.requestGoal(targetHoodAngleDeg, isScoring);
        turret.requestAngle(targetTurretAngleDeg, isScoring, targetFFRadPerSec);
        tunnel.requestIdle();
      }
      case SHOOT -> {
        if (Constants.currentMode == Mode.REAL) {
          if (tunnel.fuelDetected()) {
            if (shootingTimeoutTimer.isRunning()) {
              shootingTimeoutTimer.reset();
            } else {
              shootingTimeoutTimer.start();
              isShooting = true;
            }
          }
        } else {
          if (!simShootingTimer.isRunning()) {
            simShootingTimer.reset();
            simShootingTimer.start();
            isShooting = true;
          }
          if (simShootingTimer.hasElapsed(Constants.Tunnel.simShootingTime)) {
            isShooting = false;
          }
        }
        
        if (shootingTimeoutTimer.hasElapsed(Constants.Tunnel.shootingTimeout) && !tunnel.fuelDetected()) {
          shootingTimeoutTimer.stop();
          shootingTimeoutTimer.reset();
          isShooting = false; 
        }
        resetIdleTimeout = true;
        flywheel.requestGoal(targetFlywheelSpeedRPS, isScoring);
        hood.requestGoal(targetHoodAngleDeg, isScoring);
        turret.requestAngle(targetTurretAngleDeg, isScoring, targetFFRadPerSec);
        tunnel.requestGoal(targetTunnelSpeedRPS);
        spindexer.requestGoal(targetSpindexerSpeedRPS);
      }
    }

    flywheel.outputsPeriodic();
    spindexer.outputsPeriodic();
    tunnel.outputsPeriodic();
    hood.outputsPeriodic();

    if (!Constants.turretLocked) {
      turret.outputsPeriodic();
    }

    Logger.recordOutput("Shooter/State", state.toString());
    Logger.recordOutput("Shooter/TargetHoodAngleDeg", targetHoodAngleDeg);
    Logger.recordOutput("Shooter/TargetFlywheelSpeedRPS", targetFlywheelSpeedRPS);
    Logger.recordOutput("Shooter/TargetTurretAngleDeg", targetTurretAngleDeg);
    Logger.recordOutput("Shooter/TargetTunnelSpeedRPS", targetTunnelSpeedRPS);
    Logger.recordOutput("Shooter/TargetIndexerSpeedRPS", targetSpindexerSpeedRPS);
    Logger.recordOutput("Shooter/Turret/poseRotated", drive.getTurretPose(turret.getAngle()));
    Logger.recordOutput("Shooter/IsShooting", isShooting);
    Logger.recordOutput("Shooter/Tunnel/IsFuelDetected", isFuelDetected());
  }

  private void calculateFiringSolution() {
    if (fixedPositionShooting) {
      targetHoodAngleDeg = Constants.fixedSolutionBlue.hoodAngle;
      targetFlywheelSpeedRPS = Constants.fixedSolutionBlue.flywheelSpeedRPS;
      targetTunnelSpeedRPS = Constants.Tunnel.shootRPS;
      targetSpindexerSpeedRPS = Constants.Spindexer.shootRPS;
      targetFFRadPerSec = 0;
      if (Robot.alliance == Alliance.BLUE) {
        targetTurretAngleDeg = Constants.fixedSolutionBlue.turretAngleDeg;
      } else {
        targetTurretAngleDeg = Constants.fixedSolutionRed.turretAngleDeg;
      }
    } else {
      Translation2d shootTarget = FiringManager.getShootingTarget(drive.getTurretPose());
      Translation2d shootForward;
      ShotCalculator.ShotInputs inputs;

      if (isScoring) {
        if (Robot.alliance == Alliance.RED) {
          shootForward = new Translation2d(-1, 0);
        } else {
          shootForward = new Translation2d(1, 0);
        }
      } else {
        if (Robot.alliance == Alliance.RED) {
          shootForward = new Translation2d(1, 0);
        } else {
          shootForward = new Translation2d(-1, 0);
        }
      }

      inputs =
          new ShotCalculator.ShotInputs(
              drive.getRobotPose(),
              drive.getFieldRelativeVelocity(),
              drive.getRobotRelativeVelocity(),
              shootTarget,
              shootForward,
              0.9, // vision confidence, 0 to 1
              0, // pitch for tilt gate (0.0 if no gyro)
              0 // roll for tilt gate (0.0 if no gyro)
              );

      ShotCalculator.LaunchParameters shot;
      if (isScoring) {
        if (!hasBeenScoring) {
          // Reset stale accel/tof values in calculator upon switching to scoring
          scoreCalc.resetWarmStart();
          hasBeenScoring = true;
        }
        shot = scoreCalc.calculate(inputs);
      } else {
        if (hasBeenScoring) {
          // Reset stale accel/tof values in calculator upon switching to passing
          passCalc.resetWarmStart();
          hasBeenScoring = false;
        }
        shot = passCalc.calculate(inputs);
      }

      if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
        targetHoodAngleDeg = tunableHoodAngle.get();
        targetFlywheelSpeedRPS = tunableFlywheelSpeedRPS.get();
        targetFFRadPerSec = shot.turretAngularVelocityRadPerSec();
        targetTunnelSpeedRPS = tunableTunnelSpeedRPS.get();
        targetSpindexerSpeedRPS = tunableIndexerSpeedRPS.get();
        targetTurretAngleDeg = shot.turretAngle().getDegrees();
      } else if (shot.isValid()) {
        targetHoodAngleDeg = shot.hoodAngle();
        targetFlywheelSpeedRPS = shot.rpm() / 60.0;
        targetTurretAngleDeg = shot.turretAngle().getDegrees();
        targetFFRadPerSec = shot.turretAngularVelocityRadPerSec();
        targetTunnelSpeedRPS = Constants.Tunnel.shootRPS;
        targetSpindexerSpeedRPS = Constants.Spindexer.shootRPS;
      }
    }
  }

  public ShooterState getState() {
    return state;
  }

  public boolean hoodAtGoal() {
    return hood.isAtGoal();
  }

  public boolean isHoodLowered() {
    return hood.isSafeAngle();
  }

  // Needs to be continuously called in order to start shooting balls
  public void requestShoot(boolean fixedPosition, boolean isScoring) {
    this.isScoring = isScoring;
    this.fixedPositionShooting = fixedPosition;
    Logger.recordOutput("Shooter/currentMethod", "requestShoot()");
    if (Constants.firingManagerMode == Constants.SubsystemMode.TUNING) {
      return;
    }

    if (state != ShooterState.SHOOT) {
      if (state == ShooterState.PRESHOOT) {
        if (hood.isAtGoal() && flywheel.isAtGoal() && turret.isAtGoal()) {
          if (Constants.currentMode == Mode.SIM) {
            simShootingTimer.stop();
            simShootingTimer.reset();
          }
          state = ShooterState.SHOOT;
        }
      } else {
        // Never set requested state to SHOOT to ensure after turret unwinds
        // we wait for everything to get to setpoint before starting to shoot again
        state = ShooterState.PRESHOOT;
        requestedState = ShooterState.PRESHOOT;
      }
    }
  }

  public void requestIdle(
      Double hoodOverrideDegree, Double flywheelOverideRPS, Double turretOverrideDegree) {
    this.hoodOverrideDeg = hoodOverrideDegree;
    this.flywheelOverrideRPS = flywheelOverideRPS;
    this.turretOverrideDeg = turretOverrideDegree;
    requestedState = ShooterState.IDLE;
    state = ShooterState.IDLE;
    Logger.recordOutput("Shooter/currentMethod", "requestIdle()");
  }

  public void requestStop() {
    requestedState = ShooterState.STOP;
    state = ShooterState.STOP;
    Logger.recordOutput("Shooter/currentMethod", "requestStop()");
  }

  public void unjamOverride(boolean unjamOverride) {
    tunnel.unjamOverride(unjamOverride);
    spindexer.unjamOverride(unjamOverride);
  }

  public void trenchOverride(boolean unjamOverride) {
    hood.trenchOverride(unjamOverride);
  }

  public boolean isScoring() {
    return isScoring;
  }

  public double getHoodPositionDegrees() {
    return hood.getPositionDegrees();
  }

  public void turretUnjamOverride(boolean override) {
    turret.unjamOverride(override);
  }

  public boolean isFuelDetected() {
    return Constants.currentMode == Mode.REAL ? tunnel.fuelDetected() : isShooting;
  }

  public boolean isShooting() {
    return isShooting;
  }
}
