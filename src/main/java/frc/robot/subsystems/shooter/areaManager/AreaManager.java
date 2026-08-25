package frc.robot.subsystems.shooter.areaManager;

import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.Alliance;
import frc.robot.Robot;
import frc.robot.constants.FieldConstants;

public class AreaManager {

  public enum Zone {
    ALLIANCE_ZONE,
    LEFT_OPPOSITION,
    RIGHT_OPPOSITION,
    LEFT_NEUTRAL,
    RIGHT_NEUTRAL,
    LEFT_STOP_SHOOT,
    RIGHT_STOP_SHOOT,
    PASS_RIGHT,
    PASS_LEFT,
    LEFT_OPPOSITION_STOP_SHOOT,
    RIGHT_OPPOSITION_STOP_SHOOT,
    UNKNOWN
  }

  public static boolean isShootingArea(Pose2d position) {
    if (Robot.alliance == Alliance.BLUE) {
      return !(FieldConstants.Red.frontOfHub.contains(position)
          // Not in neutral side of blue trench
          || FieldConstants.Blue.stopShootLeftNeutral.contains(position)
          || FieldConstants.Blue.stopShootRightNeutral.contains(position)
          // Not anywhere near red trench (neutral/red alliance side)
          || FieldConstants.Red.stopShootLeftFull.contains(position)
          || FieldConstants.Red.stopShootRightFull.contains(position));
    } else {
      return !(FieldConstants.Blue.frontOfHub.contains(position)
          // Not in neutral side of red trench
          || FieldConstants.Red.stopShootLeftNeutral.contains(position)
          || FieldConstants.Red.stopShootRightNeutral.contains(position)
          // Not anywhere near blue trench (neutral/blue alliance side)
          || FieldConstants.Blue.stopShootLeftFull.contains(position)
          || FieldConstants.Blue.stopShootRightFull.contains(position));
    }
  }

  public static Zone getZoneOfPosition(Pose2d position) {
    if (Robot.alliance == Alliance.BLUE) {
      if (FieldConstants.Blue.stopShootLeftNeutral.contains(position)) {
        return Zone.LEFT_STOP_SHOOT;
      } else if (FieldConstants.Blue.stopShootRightNeutral.contains(position)) {
        return Zone.RIGHT_STOP_SHOOT;
      } else if (FieldConstants.Red.stopShootLeftFull.contains(position)) {
        return Zone.LEFT_OPPOSITION_STOP_SHOOT;
      } else if (FieldConstants.Red.stopShootRightFull.contains(position)) {
        return Zone.RIGHT_OPPOSITION_STOP_SHOOT;
      } else if (FieldConstants.Blue.allianceZone.contains(position)) {
        return Zone.ALLIANCE_ZONE;
      } else if (FieldConstants.Red.rightAllianceZone.contains(position)) {
        return Zone.RIGHT_OPPOSITION;
      } else if (FieldConstants.Red.leftAllianceZone.contains(position)) {
        return Zone.LEFT_OPPOSITION;
      } else if (FieldConstants.Blue.passLeftZone.contains(position)) {
        return Zone.PASS_LEFT;
      } else if (FieldConstants.Blue.passRightZone.contains(position)) {
        return Zone.PASS_RIGHT;
      }
    } else {
      if (FieldConstants.Red.stopShootLeftNeutral.contains(position)) {
        return Zone.LEFT_STOP_SHOOT;
      } else if (FieldConstants.Red.stopShootRightNeutral.contains(position)) {
        return Zone.RIGHT_STOP_SHOOT;
      } else if (FieldConstants.Blue.stopShootLeftFull.contains(position)) {
        return Zone.LEFT_OPPOSITION_STOP_SHOOT;
      } else if (FieldConstants.Blue.stopShootRightFull.contains(position)) {
        return Zone.RIGHT_OPPOSITION_STOP_SHOOT;
      } else if (FieldConstants.Red.allianceZone.contains(position)) {
        return Zone.ALLIANCE_ZONE;
      } else if (FieldConstants.Blue.rightAllianceZone.contains(position)) {
        return Zone.RIGHT_OPPOSITION;
      } else if (FieldConstants.Blue.leftAllianceZone.contains(position)) {
        return Zone.LEFT_OPPOSITION;
      } else if (FieldConstants.Red.passLeftZone.contains(position)) {
        return Zone.PASS_LEFT;
      } else if (FieldConstants.Red.passRightZone.contains(position)) {
        return Zone.PASS_RIGHT;
      }
    }
    return Zone.UNKNOWN;
  }

  // Returns true if positions are same larger zone, ignoring right/left designations (ex. right
  // neutral and left neutral -> true)
  public static boolean isSameCompleteZone(Pose2d position1, Pose2d position2) {
    Zone zone1 = getZoneOfPosition(position1);
    Zone zone2 = getZoneOfPosition(position2);

    if (zone1 == Zone.ALLIANCE_ZONE && zone2 == Zone.ALLIANCE_ZONE) {
      return true;
    } else if ((zone1 == Zone.LEFT_OPPOSITION || zone1 == Zone.RIGHT_OPPOSITION)
        && (zone2 == Zone.LEFT_OPPOSITION || zone2 == Zone.RIGHT_OPPOSITION)) {
      return true;
    } else if ((zone1 == Zone.LEFT_NEUTRAL || zone1 == Zone.RIGHT_NEUTRAL)
        && (zone2 == Zone.LEFT_NEUTRAL || zone2 == Zone.RIGHT_NEUTRAL)) {
      return true;
    } else if ((zone1 == Zone.LEFT_STOP_SHOOT || zone1 == Zone.RIGHT_STOP_SHOOT)
        && (zone2 == Zone.LEFT_STOP_SHOOT || zone2 == Zone.RIGHT_STOP_SHOOT)) {
      return true;
    } else if ((zone1 == Zone.LEFT_OPPOSITION_STOP_SHOOT
            || zone1 == Zone.RIGHT_OPPOSITION_STOP_SHOOT)
        && (zone2 == Zone.LEFT_OPPOSITION_STOP_SHOOT
            || zone2 == Zone.RIGHT_OPPOSITION_STOP_SHOOT)) {
      return true;
    }
    return false;
  }
}
