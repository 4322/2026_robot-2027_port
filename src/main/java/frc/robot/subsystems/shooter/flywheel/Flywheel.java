package frc.robot.subsystems.shooter.flywheel;

import org.wpilib.system.Timer;
import org.wpilib.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Flywheel {

    private FlywheelIO io;

    private FlywheelIOInputsAutoLogged inputs = new FlywheelIOInputsAutoLogged();

    private double requestedSetpoint = 0;

    private Timer atGoalTimer = new Timer();

    private boolean isScoring = false;

    public Flywheel(FlywheelIO io) {
        this.io = io;
    }

    public void inputsPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Flywheel", inputs);
    }

    public void outputsPeriodic() {
        switch(Constants.flywheelMode) {
            case TUNING ->
                {
                }
            case NORMAL ->
                {
                    updateAtGoalTimer();
                    updateNetworkTableValues();
                    Logger.recordOutput("Shooter/Flywheel/atGoal", isAtGoal());
                }
            case DISABLED ->
                {
                }
        }
    }

    public void requestGoal(double velocity, boolean isScoring) {
        switch(Constants.flywheelMode) {
            case TUNING ->
                {
                }
            case NORMAL ->
                {
                    io.setTargetMechanismRPS(velocity);
                    requestedSetpoint = velocity;
                    updateAtGoalTimer();
                }
            case DISABLED ->
                {
                }
        }
        this.isScoring = isScoring;
        Logger.recordOutput("Shooter/Flywheel/RequestedSetpoint", requestedSetpoint);
    }

    public boolean isAtGoal() {
        return Math.abs(inputs.leaderMechanismRPS - requestedSetpoint) < Constants.Flywheel.smallToleranceRPS || (isScoring ? atGoalTimer.hasElapsed(Constants.scoringDoubleToleranceTime) : atGoalTimer.hasElapsed(Constants.passingDoubleToleranceTime));
    }

    private void updateAtGoalTimer() {
        if (Math.abs(inputs.leaderMechanismRPS - requestedSetpoint) < Constants.Flywheel.largeToleranceRPS) {
            atGoalTimer.start();
        } else {
            atGoalTimer.stop();
            atGoalTimer.reset();
        }
    }

    private void updateNetworkTableValues() {
        if (isAtGoal()) {
            if (Math.abs(inputs.leaderMechanismRPS - requestedSetpoint) < Constants.Flywheel.smallToleranceRPS) {
                SmartDashboard.putString("Flywheel/FlywheelAtGoal", Constants.NetworkTables.green.toHexString());
            } else {
                SmartDashboard.putString("Flywheel/FlywheelAtGoal", Constants.NetworkTables.yellow.toHexString());
            }
        } else {
            SmartDashboard.putString("Flywheel/FlywheelAtGoal", Constants.NetworkTables.red.toHexString());
        }
    }
}
