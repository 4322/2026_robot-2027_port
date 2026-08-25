package frc.robot.subsystems.shooter.spindexer;

import org.wpilib.driverstation.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Spindexer {

    private SpindexerIO io;

    private SpindexerIOInputsAutoLogged inputs = new SpindexerIOInputsAutoLogged();

    private double requestedSpeed = -1;

    private boolean unjamOverride;

    public enum SpindexerStates {

        DISABLED, IDLE, INDEXING, UNJAM
    }

    private SpindexerStates state = SpindexerStates.DISABLED;

    public Spindexer(SpindexerIO io) {
        this.io = io;
    }

    public void inputsPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Spindexer", inputs);
    }

    public void outputsPeriodic() {
        switch(Constants.spindexerMode) {
            case TUNING ->
                {
                }
            case NORMAL ->
                {
                    if (org.wpilib.driverstation.RobotState.isDisabled()) {
                        state = SpindexerStates.DISABLED;
                    }
                    switch(state) {
                        case DISABLED ->
                            {
                                // Reset variables
                                unjamOverride = false;
                                if (org.wpilib.driverstation.RobotState.isEnabled()) {
                                    state = SpindexerStates.IDLE;
                                }
                            }
                        case IDLE ->
                            {
                                if (inputs.leaderMechanismRPS < Constants.Spindexer.stopTreshold || requestedSpeed == 0) {
                                    io.stop();
                                    requestedSpeed = 0;
                                } else {
                                    requestedSpeed = 0.5;
                                    io.setTargetMechanismRotations(requestedSpeed);
                                }
                            }
                        case INDEXING ->
                            {
                                io.setTargetMechanismRotations(requestedSpeed);
                            }
                        case UNJAM ->
                            {
                                io.setTargetMechanismRotations(Constants.Spindexer.unjamRPS);
                            }
                    }
                }
            case DISABLED ->
                {
                }
        }
        Logger.recordOutput("Shooter/Spindexer/State", state.toString());
        Logger.recordOutput("Shooter/Spindexer/RequestedSpeed", requestedSpeed);
        Logger.recordOutput("Shooter/Spindexer/Stopped", isStopped());
    }

    public void requestIdle() {
        if (!unjamOverride) {
            state = SpindexerStates.IDLE;
        }
    }

    public void requestGoal(double speed) {
        if (!unjamOverride) {
            state = SpindexerStates.INDEXING;
            requestedSpeed = speed;
        }
    }

    public void unjamOverride(boolean unjamOverride) {
        this.unjamOverride = unjamOverride;
        if (unjamOverride) {
            state = SpindexerStates.UNJAM;
            requestedSpeed = Constants.Spindexer.unjamRPS;
        } else {
            // Default to idle when unjam isn't desired
            state = SpindexerStates.IDLE;
            requestedSpeed = 0;
        }
    }

    public void enableBrakeMode(boolean enable) {
        io.enableBrakeMode(enable);
    }

    public boolean isStopped() {
        return inputs.leaderMechanismRPS < Constants.Spindexer.stoppedMechanismRotationsPerSec;
    }

    public double getVelocity() {
        return inputs.leaderMechanismRPS;
    }
}
