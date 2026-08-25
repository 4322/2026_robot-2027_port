package frc.robot.autonomous.modes;

import org.wpilib.command3.Command;
import org.wpilib.opmode.Autonomous;
import org.wpilib.opmode.PeriodicOpMode;

import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystems.shooter.hood.Hood;

@Autonomous(name = "Do Nothing", group = "Misc")
public class DoNothing extends PeriodicOpMode {
  private final Robot robot;

  public DoNothing(Robot robot) {
    this.robot = robot;
  }

  public void start() {
    RobotContainer.hood.requestGoal(0, false);
  }
}
