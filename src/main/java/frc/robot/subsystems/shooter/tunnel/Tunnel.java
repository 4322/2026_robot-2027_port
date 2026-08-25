package frc.robot.subsystems.shooter.tunnel;

import org.wpilib.driverstation.DriverStation;
import frc.robot.constants.Constants;
import org.littletonrobotics.junction.Logger;

public class Tunnel {

    private TunnelIO io;

    private TunnelIOInputsAutoLogged inputs = new TunnelIOInputsAutoLogged();

    private double requestedSpeed = -1;

    private boolean unjamOverride;

    public enum TunnelStates {

        DISABLED, IDLE, INDEXING, UNJAM
    }

    private TunnelStates state = TunnelStates.DISABLED;

    public Tunnel(TunnelIO io) {
        this.io = io;
    }

    public void inputsPeriodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Shooter/Tunnel", inputs);
    }

    public void outputsPeriodic() {
        switch(Constants.tunnelMode) {
            case TUNING ->
                {
                }
            case DISABLED ->
                {
                }
            case NORMAL ->
                {
                    if (org.wpilib.driverstation.RobotState.isDisabled()) {
                        state = TunnelStates.DISABLED;
                    }
                    switch(state) {
                        case DISABLED ->
                            {
                                // Reset variables
                                unjamOverride = false;
                                if (org.wpilib.driverstation.RobotState.isEnabled()) {
                                    state = TunnelStates.IDLE;
                                }
                            }
                        case IDLE ->
                            {
                                io.stop();
                            }
                        case INDEXING ->
                            {
                                io.setTargetMechanismRotations(requestedSpeed);
                            }
                        case UNJAM ->
                            {
                                io.setTargetMechanismRotations(Constants.Tunnel.unjamRPS);
                            }
                    }
                }
        }
        Logger.recordOutput("Shooter/Tunnel/State", state.toString());
        Logger.recordOutput("Shooter/Tunnel/RequestedSpeed", requestedSpeed);
        Logger.recordOutput("Shooter/Tunnel/Stopped", isStopped());
    }

    public void requestIdle() {
        if (!unjamOverride) {
            state = TunnelStates.IDLE;
            requestedSpeed = 0;
        }
    }

    public void unjamOverride(boolean unjamOverride) {
        this.unjamOverride = unjamOverride;
        if (unjamOverride) {
            state = TunnelStates.UNJAM;
            requestedSpeed = Constants.Tunnel.unjamRPS;
        } else {
            // Default to idle when unjam isn't desired
            state = TunnelStates.IDLE;
            requestedSpeed = 0;
        }
    }

    public void requestGoal(double speed) {
        if (!unjamOverride) {
            state = TunnelStates.INDEXING;
            requestedSpeed = speed;
        }
    }

    public void enableBrakeMode(boolean enable) {
        io.enableBrakeMode(enable);
    }

    public boolean isStopped() {
        return inputs.mechanismRPS < Constants.Tunnel.stoppedMechanismRotationsPerSec;
    }

    public double getVelocity() {
        return inputs.mechanismRPS;
    }
}
