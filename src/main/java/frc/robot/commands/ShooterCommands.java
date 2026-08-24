package frc.robot.commands;

import org.wpilib.math.geometry.Rotation2d;

import static org.wpilib.units.Units.Seconds;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command3.Command;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;

public class ShooterCommands {

  public static Command autoShoot(Shooter shooter, Drive drive, Intake intake) {
    return Command.noRequirements(coroutine -> {
      if (intake.hasExtended()) {
        coroutine.await(Command.parallel(
            DriveCommands.joystickDriveWhileShooting(
                drive,
                () -> -RobotContainer.controller.getLeftY(),
                () -> -RobotContainer.controller.getLeftX(),
                () -> -RobotContainer.controller.getRightX(),
                () -> shooter.isScoring()),
            shoot(shooter, drive)).withAutomaticName());
      }
    })
        .named("autoShoot");
  }

  /*
   * Old Commands V2 code
   * public static Command autoShoot(Shooter shooter, Drive drive, Intake intake)
   * {
   * return Commands.parallel(
   * DriveCommands.joystickDriveWhileShooting(
   * drive,
   * () -> -RobotContainer.controller.getLeftY(),
   * () -> -RobotContainer.controller.getLeftX(),
   * () -> -RobotContainer.controller.getRightX(),
   * () -> shooter.isScoring()),
   * new Shoot(shooter, drive))
   * .onlyIf(() -> intake.hasExtended());
   * }
   */

  public static Command fixedShoot(Shooter shooter, Intake intake) {
    return Command.noRequirements(coroutine -> {
      if (intake.hasExtended()) {
        coroutine.await(shootFixed(shooter));
      }
    })
        .named("fixedShoot");
  }

  /*
   * Old Commands V2 code
   * public static Command fixedShoot(Shooter shooter, Drive drive, Intake intake)
   * {
   * return new ShootFixed(shooter).onlyIf(() -> intake.hasExtended());
   * }
   */

  // Command is only used when shooter is fixed so no need for intake extended
  // check
  public static Command aimAndShoot(Shooter shooter, Drive drive) {
    return Command.parallel(
        DriveCommands.joystickDriveAtAngle(
            drive,
            () -> -RobotContainer.controller.getLeftY(),
            () -> -RobotContainer.controller.getLeftX(),
            () -> Rotation2d.fromDegrees(shooter.getTargetTurretAngleDeg()),
            Constants.Turret.originToTurret),
        shoot(shooter, drive)).withAutomaticName();
  }

  /*
   * Old Commands V2 code
   * public static Command aimAndShoot(Shooter shooter, Drive drive, Intake
   * intake) {
   * return new ParallelCommandGroup(
   * DriveCommands.joystickDriveAtAngle(
   * drive,
   * () -> -RobotContainer.controller.getLeftY(),
   * () -> -RobotContainer.controller.getLeftX(),
   * () -> Rotation2d.fromDegrees(shooter.getTargetTurretAngleDeg()),
   * Constants.Turret.originToTurret),
   * new Shoot(shooter, drive));
   * }
   */

  public static Command idle(
      Shooter shooter, Intake intake, Double hoodAngle, Double flywheelRPS, Double turretAngle) {
    return Command.noRequirements(coroutine -> {
      if (intake.hasExtended()) {
        coroutine.await(shooter.runRepeatedly(() -> {
          shooter.requestIdle(hoodAngle, flywheelRPS, turretAngle);
        }).named("requestIdle"));
      }
    }).named("idle");
  }

  /*
   * Old Commands V2 code
   * public static Command idle(
   * Shooter shooter, Intake intake, Double hoodAngle, Double flywheelRPS, Double
   * turretAngle) {
   * return Commands.run(
   * () -> {
   * shooter.requestIdle(hoodAngle, flywheelRPS, turretAngle);
   * },
   * shooter)
   * .onlyIf(() -> intake.hasExtended());
   * }
   */

  public static Command stop(Shooter shooter) {
    return shooter.run(coroutine -> {
      shooter.requestStop();
    }).named("stop");
  }

  /*
   * Old Commands V2 code
   * public static Command stop(Shooter shooter) {
   * return Commands.run(
   * () -> {
   * shooter.requestStop();
   * },
   * shooter);
   * }
   */

  // Don't require shooter since this is an override meant to be done anytime
  public static Command trenchOverride(Shooter shooter) {
    return shooter.runRepeatedly(() -> shooter.trenchOverride(true))
        .whenCanceled(() -> shooter.trenchOverride(false))
        .named("trenchOverride");
  }
  /*
   * Old Commands V2 code
   * public static Command trenchOverride(Shooter shooter) {
   * return Commands.run(
   * () -> {
   * shooter.trenchOverride(true);
   * })
   * .finallyDo(() -> shooter.trenchOverride(false));
   * }
   */

  public static Command turretUnjamOverride(Shooter shooter, boolean override) {
    return Command.noRequirements(coroutine -> {
      if (!(shooter.getState() == Shooter.ShooterState.SHOOT
                || shooter.getState() == Shooter.ShooterState.PRESHOOT)) {
                  shooter.turretUnjamOverride(override);
                }
    }).named("turretUnjamOverride");
  }

  /* Old Commands V2 code
  public static Command turretUnjamOverride(Shooter shooter, boolean override) {
    return Commands.runOnce(
        () -> {
          shooter.turretUnjamOverride(override);
        })
        .onlyIf(
            () -> !(shooter.getState() == Shooter.ShooterState.SHOOT
                || shooter.getState() == Shooter.ShooterState.PRESHOOT));
  }
                */

  // Don't require shooter since this is an override meant to be done anytime
  public static Command unjam(Shooter shooter) {
    return Command.noRequirements(coroutine -> {
      while (true) {
        shooter.unjamOverride(true);
      }
    })
    .whenCanceled(() -> shooter.unjamOverride(false))
    .named("unjam");
  }

  /* Old Commands V2 code
  public static Command unjam(Shooter shooter) {
    return Commands.run(
        () -> {
          shooter.unjamOverride(true);
        })
        .finallyDo(() -> shooter.unjamOverride(false));
  }
  */

  public static Command shoot(Shooter shooter, Drive drive) {
    return shoot(shooter, drive, false);
  }

  public static Command shoot(Shooter shooter, Drive drive, boolean ignoreArea) {
    return shooter.runRepeatedly(() -> {
      Logger.recordOutput(
          "Shooter/currentZone", AreaManager.getZoneOfPosition(drive.getTurretPose()).toString());
      if ((!AreaManager.isShootingArea(drive.getTurretPose())) && !ignoreArea) {
        shooter.requestIdle(null, null, null);
      } else {
        if (AreaManager.getZoneOfPosition(drive.getTurretPose()) == Zone.ALLIANCE_ZONE) {
          shooter.requestShoot(false, true);
        } else {
          shooter.requestShoot(false, false);
        }
      }
    })
        .whenCanceled(() -> shooter.requestIdle(null, null, null))
        .named("shoot");
  }

  /*
   * Old Commands V2 code
   * 
   * public class Shoot extends Command {
   * private Shooter shooter;
   * private Drive drive;
   * private boolean ignoreArea;
   * 
   * public Shoot(Shooter shooter, Drive drive) {
   * this.shooter = shooter;
   * this.drive = drive;
   * this.ignoreArea = false;
   * }
   * 
   * public Shoot(Shooter shooter, Drive drive, boolean ignoreArea) {
   * this.shooter = shooter;
   * this.drive = drive;
   * this.ignoreArea = ignoreArea;
   * }
   * 
   * @Override
   * public void execute() {
   * Logger.recordOutput(
   * "Shooter/currentZone",
   * AreaManager.getZoneOfPosition(drive.getTurretPose()).toString());
   * if ((!AreaManager.isShootingArea(drive.getTurretPose())) && !ignoreArea) {
   * shooter.requestIdle(null, null, null);
   * } else {
   * if (AreaManager.getZoneOfPosition(drive.getTurretPose()) ==
   * Zone.ALLIANCE_ZONE) {
   * shooter.requestShoot(false, true);
   * } else {
   * shooter.requestShoot(false, false);
   * }
   * }
   * }
   * 
   * @Override
   * public void end(boolean interrupted) {
   * shooter.requestIdle(null, null, null);
   * }
   * }
   */

  public static Command shootFixed(Shooter shooter) {
    return shooter.runRepeatedly(() -> {
      shooter.requestShoot(true, true);
    })
        .whenCanceled(() -> shooter.requestIdle(null, null, null))
        .named("shootFixed");
  }

  /*
   * Old Commands V2 code
   * public class ShootFixed extends Command {
   * private Shooter shooter;
   * 
   * public ShootFixed(Shooter shooter) {
   * this.shooter = shooter;
   * }
   * 
   * @Override
   * public void execute() {
   * shooter.requestShoot(true, true);
   * }
   * 
   * @Override
   * public void end(boolean interrupted) {
   * shooter.requestIdle(null, null, null);
   * }
   * }
   */

  // Below commands used in auto ONLY

  public static Command autoShootNoAreaCheck(Shooter shooter, Drive drive, Intake intake) {
    return Command.noRequirements(coroutine -> {
      if (intake.hasExtended()) {
        coroutine.await(shoot(shooter, drive, true));
      }
    }).named("autoShootNoAreaCheck");
  }

  /* Old Commands V2 code
  public static Command autoShootNoAreaCheck(Shooter shooter, Drive drive, Intake intake) {
    return new Shoot(shooter, drive, true).onlyIf(() -> intake.hasExtended());
  }
  */

  public static Command autoShootWithAreaCheck(Shooter shooter, Drive drive, Intake intake) {
    return Command.noRequirements(coroutine -> {
      if (intake.hasExtended()) {
        coroutine.await(shoot(shooter, drive));
      }
    }).named("autoShootNoAreaCheck");
  }

  /* Old Commands V2 code
  public static Command autoShootWithAreaCheck(Shooter shooter, Drive drive, Intake intake) {
    return new Shoot(shooter, drive).onlyIf(() -> intake.hasExtended());
  }
    */

  public static Command autoUnjam(Shooter shooter, double timeout) {
    return Command.noRequirements(coroutine -> {
      while (true) {
        shooter.unjamOverride(true);
      }
    })
    .whenCanceled(() -> shooter.unjamOverride(false))
    .named("autoUnjam")
    .withTimeout(Seconds.of(timeout));
  }

/* Old Commands V2 code
  public static Command autoUnjam(Shooter shooter, double timeout) {
    return Commands.run(
        () -> {
          shooter.unjamOverride(true);
        })
        .finallyDo(() -> shooter.unjamOverride(false))
        .withTimeout(timeout);
  }
        */
}
