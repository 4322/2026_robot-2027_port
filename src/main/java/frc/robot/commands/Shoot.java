package frc.robot.commands;

import org.wpilib.command2.Command;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;
import org.littletonrobotics.junction.Logger;

public class Shoot extends Command {
  private Shooter shooter;
  private Drive drive;
  private boolean ignoreArea;

  public Shoot(Shooter shooter, Drive drive) {
    this.shooter = shooter;
    this.drive = drive;
    this.ignoreArea = false;
  }

  public Shoot(Shooter shooter, Drive drive, boolean ignoreArea) {
    this.shooter = shooter;
    this.drive = drive;
    this.ignoreArea = ignoreArea;
  }

  @Override
  public void execute() {
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
  }

  @Override
  public void end(boolean interrupted) {
    shooter.requestIdle(null, null, null);
  }
}
