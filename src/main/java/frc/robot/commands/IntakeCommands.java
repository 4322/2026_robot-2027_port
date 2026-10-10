package frc.robot.commands;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;

import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;

public class IntakeCommands {

  public static Command idle(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.IDLE);
            })
        .onlyIf(() -> intake.hasExtended());
  }

  public static Command autoIntake(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.setState(IntakeState.INTAKING);
        });
  }

 public static Command manualIntake(Intake intake) {
    return Commands.runOnce(
        () -> {
          intake.setState(IntakeState.INTAKING);
        })
            .onlyIf(
                () ->
                    intake.getState() == IntakeState.IDLE
                        || intake.getState() == IntakeState.STARING_CONFIG);
  }


  public static Command eject(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.EJECT);
            })
        .onlyIf(() -> intake.isExtended());
  }

  public static Command smoosh(Intake intake) {
    return Commands.runOnce(
            () -> {
              intake.setState(IntakeState.SMOOSH);
            })
        .onlyIf(() -> intake.hasExtended());
  }

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

  public static Command autoSmoosh(Intake intake) {
    return Commands.run(
            () -> {
              intake.setState(IntakeState.SMOOSH);
            })
        .onlyIf(() -> intake.hasExtended());
  }

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
}
