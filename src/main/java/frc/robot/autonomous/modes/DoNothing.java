package frc.robot.autonomous.modes;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.subsystems.shooter.hood.Hood;
import org.littletonrobotics.junction.Logger;

public class DoNothing extends SequentialCommandGroup {

    public DoNothing(Hood hood) {
        setName("DO_NOTHING");
        addCommands(new InstantCommand(() -> hood.requestGoal(0.0, false)), new InstantCommand(() -> Logger.recordOutput("Autonomous/AutoStarted", "DoNothing")));
    }
}
