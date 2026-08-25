package frc.robot.autonomous.modes;

import org.wpilib.command3.Command;
import frc.robot.subsystems.shooter.hood.Hood;

public class DoNothing {

  public static Command auto(Hood hood) {
    return Command.sequence(
      Command.noRequirements(coroutine -> hood.requestGoal(0.0, false)).named("hoodSetGoal")
    ).named("DO_NOTHING");
  }

}
