package frc.robot.subsystems.shooter.firingManager;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.driverstation.Alliance;
import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.subsystems.shooter.areaManager.AreaManager.Zone;
import org.littletonrobotics.junction.Logger;

public class FiringManager {

  public enum FiringTargets {
    HUB,
    ALLIANCE_RIGHT,
    ALLIANCE_LEFT,
    NEUTRAL_LEFT,
    NEUTRAL_RIGHT
  }

  public static Translation2d getShootingTarget(Pose2d robotPosition) {
    Zone zone = AreaManager.getZoneOfPosition(robotPosition);

    if (Robot.alliance == Alliance.BLUE) {

      switch (zone) {
        case ALLIANCE_ZONE:
          Logger.recordOutput("FiringManager/targetZone", "Hub");
          return Constants.FiringTargetTranslations.Blue.hubTranslation;
        case PASS_RIGHT:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
          return Constants.FiringTargetTranslations.Blue.allianceLeftTranslation;
        case PASS_LEFT:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
          return Constants.FiringTargetTranslations.Blue.allianceRightTranslation;
        case RIGHT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
            return Constants.FiringTargetTranslations.Blue.allianceRightTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Right");
            return Constants.FiringTargetTranslations.Blue.neutralRightTranslation;
          }
        case LEFT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
            return Constants.FiringTargetTranslations.Blue.allianceLeftTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Left");
            return Constants.FiringTargetTranslations.Blue.neutralLeftTranslation;
          }
        default:
          Logger.recordOutput("FiringManager/targetZone", "Invalid (Targeting Hub)");
          return Constants.FiringTargetTranslations.Blue.hubTranslation;
      }
    } else {
      switch (zone) {
        case ALLIANCE_ZONE:
          Logger.recordOutput("FiringManager/targetZone", "Hub");
          return Constants.FiringTargetTranslations.Red.hubTranslation;
        case PASS_RIGHT:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
          return Constants.FiringTargetTranslations.Red.allianceRightTranslation;
        case PASS_LEFT:
          Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
          return Constants.FiringTargetTranslations.Red.allianceLeftTranslation;
        case RIGHT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Right");
            return Constants.FiringTargetTranslations.Red.allianceRightTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Right");
            return Constants.FiringTargetTranslations.Red.neutralRightTranslation;
          }
        case LEFT_OPPOSITION:
          if (Constants.FiringManager.alwaysTargetAllianceZone) {
            Logger.recordOutput("FiringManager/targetZone", "Alliance Left");
            return Constants.FiringTargetTranslations.Red.allianceLeftTranslation;
          } else {
            Logger.recordOutput("FiringManager/targetZone", "Neutral Left");
            return Constants.FiringTargetTranslations.Red.neutralLeftTranslation;
          }
        default:
          Logger.recordOutput("FiringManager/targetZone", "Invalid (Targeting Hub)");
          return Constants.FiringTargetTranslations.Red.hubTranslation;
      }
    }
  }
}
