package frc.robot.commands;

import org.wpilib.driverstation.GenericHID;
import org.wpilib.command3.Command;
import org.wpilib.command3.button.CommandXboxController;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.Intake.IntakeState;
import org.wpilib.command2.button.CommandNiDsXboxController;

public class IntakeCommands implements Mechanism {

    private static Command transitionToIntake(Intake intake, CommandNiDsXboxController controller) {
        return run(coroutine -> {
            coroutine.await(intake(intake));
            controller.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 0.5);
            coroutine.wait(0.25);
            controller.setRumble(GenericHID.RumbleType.LEFT_RUMBLE, 0.0);
        }).named("Transition to Intake");
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
