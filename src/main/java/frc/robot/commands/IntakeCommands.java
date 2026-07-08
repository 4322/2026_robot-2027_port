package frc.robot.commands;

import org.wpilib.driverstation.GenericHID;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandNiDsXboxController;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeCommands {
  private static Command transitionToIntake(Intake intake, CommandNiDsXboxController controller) {
    return Command.noRequirements(coroutine -> {
      coroutine.await(intake(intake));
      controller.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 0.5);
      coroutine.wait(Seconds.of(0.25));
      controller.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 0.0);
      
    })
    .named("Intake - transitionToIntake");
    
  }

  /* Original Commands v2 code

  private static Command transitionToIntake(Intake intake, CommandXboxController controller) {
    return intake(intake)
        .andThen(
            Commands.run(
                    () -> {
                      controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.5);
                    })
                .withTimeout(0.25)
                .finallyDo(
                    () -> {
                      controller.setRumble(GenericHID.RumbleType.kLeftRumble, 0.0);
                    }));
  }
   */

  public static Command idle(Intake intake) {
    return Command.noRequirements(coroutine -> {
      if (intake.hasExtended()) {
        intake.setState(IntakeState.IDLE);
      }
    })
    .named("Intake - idle");
  }

  /*  Original Commands v2 code

  public static Command idle(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.IDLE);
            })
        .onlyIf(() -> intake.hasExtended());
  }
  */

  public static Command intake(Intake intake) {
    return Command.noRequirements(coroutine -> {
      intake.setState(IntakeState.INTAKING);
    })
    .named("Intake - intake");
  }

  /* Original Commands v2 code

  public static Command intake(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.setState(IntakeState.INTAKING);
        });
  }
  */

  public static Command toggleIntake(Intake intake, CommandNiDsXboxController controller) {
    return Command.noRequirements(coroutine -> {
      if (intake.getState() == IntakeState.INTAKING) {
        coroutine.await(idle(intake));
      } else if (intake.getState() == IntakeState.IDLE || intake.getState() == IntakeState.STARING_CONFIG) {
        coroutine.await(transitionToIntake(intake, controller));
      }
    })
    .named("Intake - toggleIntake");
  }

  /* Original Commands v2 code

  public static Command toggleIntake(Intake intake, CommandXboxController controller) {
    return new ConditionalCommand(
        transitionToIntake(intake, controller)
            .onlyIf(
                () ->
                    intake.getState() == IntakeState.IDLE
                        || intake.getState() == IntakeState.STARING_CONFIG),
        idle(intake),
        () -> intake.getState() != IntakeState.INTAKING);
  }
  */

  public static Command eject(Intake intake) {
    return Command.noRequirements(coroutine -> {
      if (intake.hasExtended()) {
        intake.setState(IntakeState.EJECT);
      }
    })
    .named("Intake - eject");
  }

  /* Original Commands v2 code

  public static Command eject(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.EJECT);
            })
        .onlyIf(() -> intake.isExtended());
  }
  */

  public static Command smoosh(Intake intake) {
    return Command.noRequirements(coroutine -> {
      if (intake.hasExtended()) {
        intake.setState(IntakeState.SMOOSH);
      }
    })
    .named("Intake - smoosh");
  }

  /* Original Commands v2 code

  public static Command smoosh(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.SMOOSH);
            })
        .onlyIf(() -> intake.hasExtended());
  }
  */

  public static Command toggleOff(Intake intake) {
    return Command.noRequirements(coroutine -> {
      if (intake.hasExtended()) {
        if (intake.getPrevState() == Intake.IntakeState.INTAKING) {
          intake.setState(IntakeState.INTAKING);
        } else {
          intake.setState(IntakeState.IDLE);
        }
      }
    })
    .named("Intake - toggleOff");
  }

  /* Original Commands v2 code

  public static Command toggleOff(Intake intake) {
    return Commands.runOnce(
            () -> {
              if (intake.getPrevState() == Intake.IntakeState.INTAKING) {
                intake.setState(IntakeState.INTAKING);
              } else {
                intake.setState(IntakeState.IDLE);
              }
            })
        .onlyIf(() -> intake.hasExtended());
  }
  */
  
  public static Command autoSmoosh(Intake intake) {
    return Command.noRequirements(coroutine -> {
      if (intake.hasExtended()) {
        intake.setState(IntakeState.SMOOSH);
      }
    })
    .named("Intake - autoSmoosh");
  }

  /* Original Commands v2 code

  public static Command autoSmoosh(Intake intake) {
    return Commands.run(
            () -> {
              intake.setState(IntakeState.SMOOSH);
            })
        .onlyIf(() -> intake.hasExtended());
  }
  */

  public static Command autoSmoosh(Intake intake, double delay, double timeout) {
    return Command.noRequirements(coroutine -> {
      coroutine.wait(Seconds.of(delay));
      if (intake.hasExtended()) {
        intake.setState(IntakeState.SMOOSH);
        coroutine.wait(Seconds.of(timeout));
        coroutine.await(toggleOff(intake));
      }
    })
    .named("Intake - autoSmoosh (delay/timeout)");
  }

  /* Original Commands v2 code

  public static Command autoSmoosh(Intake intake, double delay, double timeout) {
    return new SequentialCommandGroup(
            new WaitCommand(delay),
            Commands.runOnce(
                () -> {
                  intake.setState(IntakeState.SMOOSH);
                }),
            new WaitCommand(timeout))
        .finallyDo(() -> toggleOff(intake))
        .onlyIf(() -> intake.hasExtended());
  }
  */
}
