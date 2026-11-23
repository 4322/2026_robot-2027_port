package frc.robot.sim;

import frc.robot.sim.scenario.ScenarioEvent;
import java.util.function.Consumer;

/**
 * Applies input events (buttons, sticks, poses, game data) to the virtual gamepad / robot.
 * {@code expect} events are handled by {@link Simulator} because they need timing state.
 */
final class EventExecutor {
  private final VirtualGamepad gamepad;
  private final Consumer<ScenarioEvent.SetPose> setPose;

  EventExecutor(VirtualGamepad gamepad, Consumer<ScenarioEvent.SetPose> setPose) {
    this.gamepad = gamepad;
    this.setPose = setPose;
  }

  void execute(ScenarioEvent event) {
    switch (event) {
      case ScenarioEvent.SetPose p -> setPose.accept(p);
      case ScenarioEvent.Button b -> gamepad.button(b.button(), b.action());
      case ScenarioEvent.Pov p -> gamepad.pov(p.direction(), p.action());
      case ScenarioEvent.Trigger t -> gamepad.trigger(t.side(), t.value());
      case ScenarioEvent.Drive d -> gamepad.drive(d.x(), d.y());
      case ScenarioEvent.Turn t -> gamepad.turn(t.rate());
      case ScenarioEvent.StopJoysticks s -> gamepad.stopSticks();
      case ScenarioEvent.GameData g -> gamepad.gameData(g.value());
      case ScenarioEvent.Expect x -> throw new IllegalArgumentException("expect events are handled by Simulator");
      case ScenarioEvent.End e -> {}
    }
  }
}
