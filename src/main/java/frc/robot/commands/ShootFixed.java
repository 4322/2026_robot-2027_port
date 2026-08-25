package frc.robot.commands;

import org.wpilib.command2.Command;
import frc.robot.subsystems.shooter.Shooter;

public class ShootFixed extends Command {

    private Shooter shooter;

    public ShootFixed(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void execute() {
        shooter.requestShoot(true, true);
    }

    @Override
    public void end(boolean interrupted) {
        shooter.requestIdle(null, null, null);
    }
}
