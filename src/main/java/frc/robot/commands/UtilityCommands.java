package frc.robot.commands;

import org.wpilib.system.Timer;
import org.wpilib.command2.Command;
import java.util.function.DoubleSupplier;

public class UtilityCommands {

    private UtilityCommands() {
    }

    public static class WaitSupplierCommand extends Command {

        /**
         * The timer used for waiting.
         */
        protected Timer m_timer = new Timer();

        private final DoubleSupplier m_duration;

        /**
         * Creates a new WaitCommand. This command will do nothing, and end after the specified
         * duration.
         *
         * @param seconds the time to wait, in seconds
         */
        @SuppressWarnings("this-escape")
        public WaitSupplierCommand(DoubleSupplier seconds) {
            m_duration = seconds;
        }

        @Override
        public void initialize() {
            m_timer.restart();
        }

        @Override
        public void end(boolean interrupted) {
            m_timer.stop();
        }

        @Override
        public boolean isFinished() {
            return m_timer.hasElapsed(m_duration.getAsDouble());
        }

        @Override
        public boolean runsWhenDisabled() {
            return true;
        }
    }
}
