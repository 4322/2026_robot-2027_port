package frc.robot.commands;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import frc.robot.RobotContainer;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class ShooterCommands {

    public static Command autoShoot(Shooter shooter, Drive drive, Intake intake) {
        return Commands.parallel(DriveCommands.joystickDriveWhileShooting(drive, () -> -RobotContainer.controller.getLeftY(), () -> -RobotContainer.controller.getLeftX(), () -> -RobotContainer.controller.getRightX(), () -> shooter.isScoring()), new Shoot(shooter, drive)).onlyIf(() -> intake.hasExtended());
    }

    public static Command fixedShoot(Shooter shooter, Drive drive, Intake intake) {
        return new ShootFixed(shooter).onlyIf(() -> intake.hasExtended());
    }

    // Command is only used when shooter is fixed so no need for intake extended check
    public static Command aimAndShoot(Shooter shooter, Drive drive, Intake intake) {
        return Commands.parallel(DriveCommands.joystickDriveAtAngle(drive, () -> -RobotContainer.controller.getLeftY(), () -> -RobotContainer.controller.getLeftX(), () -> Rotation2d.fromDegrees(shooter.getTargetTurretAngleDeg()), Constants.Turret.originToTurret), new Shoot(shooter, drive));
    }

    public static Command idle(Shooter shooter, Intake intake, Double hoodAngle, Double flywheelRPS, Double turretAngle) {
        return Commands.run(() -> {
            shooter.requestIdle(hoodAngle, flywheelRPS, turretAngle);
        }, shooter).onlyIf(() -> intake.hasExtended());
    }

    public static Command stop(Shooter shooter) {
        return Commands.run(() -> {
            shooter.requestStop();
        }, shooter);
    }

    // Don't require shooter since this is an override meant to be done anytime
    public static Command trenchOverride(Shooter shooter) {
        return Commands.run(() -> {
            shooter.trenchOverride(true);
        }).finallyDo(() -> shooter.trenchOverride(false));
    }

    public static Command turretUnjamOverride(Shooter shooter, boolean override) {
        return Commands.runOnce(() -> {
            shooter.turretUnjamOverride(override);
        }).onlyIf(() -> !(shooter.getState() == Shooter.ShooterState.SHOOT || shooter.getState() == Shooter.ShooterState.PRESHOOT));
    }

    // Don't require shooter since this is an override meant to be done anytime
    public static Command unjam(Shooter shooter) {
        return Commands.run(() -> {
            shooter.unjamOverride(true);
        }).finallyDo(() -> shooter.unjamOverride(false));
    }

    // Below commands used in auto ONLY
    public static Command autoShootNoAreaCheck(Shooter shooter, Drive drive, Intake intake) {
        return new Shoot(shooter, drive, true).onlyIf(() -> intake.hasExtended());
    }

    public static Command autoShootWithAreaCheck(Shooter shooter, Drive drive, Intake intake) {
        return new Shoot(shooter, drive).onlyIf(() -> intake.hasExtended());
    }

    public static Command autoUnjam(Shooter shooter, double timeout) {
        return Commands.run(() -> {
            shooter.unjamOverride(true);
        }).finallyDo(() -> shooter.unjamOverride(false)).withTimeout(timeout);
    }
}
