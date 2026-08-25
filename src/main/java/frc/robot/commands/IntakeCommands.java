package frc.robot.commands;

import org.wpilib.driverstation.GenericHID;
import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.ConditionalCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import org.wpilib.command2.button.CommandNiDsXboxController;

public class IntakeCommands {

    private static Command transitionToIntake(Intake intake, CommandNiDsXboxController controller) {
    return intake(intake)
        .andThen(
            Commands.run(
                    () -> {
                      controller.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 0.5);
                    })
                .withTimeout(0.25)
                .finallyDo(
                    () -> {
                      controller.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 0.0);
                    }));
  }

    public static Command idle(Intake intake) {
        return Commands.runOnce(() -> {
            intake.setState(IntakeState.IDLE);
        }).onlyIf(() -> intake.hasExtended());
    }

    public static Command intake(Intake intake) {
        return Commands.runOnce(() -> {
            intake.setState(IntakeState.INTAKING);
        });
    }

    public static Command toggleIntake(Intake intake, CommandNiDsXboxController controller) {
        return new ConditionalCommand(transitionToIntake(intake, controller).onlyIf(() -> intake.getState() == IntakeState.IDLE || intake.getState() == IntakeState.STARING_CONFIG), idle(intake), () -> intake.getState() != IntakeState.INTAKING);
    }

    public static Command eject(Intake intake) {
        return Commands.runOnce(() -> {
            intake.setState(IntakeState.EJECT);
        }).onlyIf(() -> intake.isExtended());
    }

    public static Command smoosh(Intake intake) {
        return Commands.runOnce(() -> {
            intake.setState(IntakeState.SMOOSH);
        }).onlyIf(() -> intake.hasExtended());
    }

    public static Command toggleOff(Intake intake) {
        return Commands.runOnce(() -> {
            if (intake.getPrevState() == Intake.IntakeState.INTAKING) {
                intake.setState(IntakeState.INTAKING);
            } else {
                intake.setState(IntakeState.IDLE);
            }
        }).onlyIf(() -> intake.hasExtended());
    }

    public static Command autoSmoosh(Intake intake) {
        return Commands.run(() -> {
            intake.setState(IntakeState.SMOOSH);
        }).onlyIf(() -> intake.hasExtended());
    }

    public static Command autoSmoosh(Intake intake, double delay, double timeout) {
        return new SequentialCommandGroup(new WaitCommand(delay), Commands.runOnce(() -> {
            intake.setState(IntakeState.SMOOSH);
        }), new WaitCommand(timeout)).finallyDo(() -> toggleOff(intake)).onlyIf(() -> intake.hasExtended());
    }
}
