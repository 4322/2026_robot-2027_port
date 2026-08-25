package frc.robot.subsystems.intake.deployer;

import org.wpilib.system.Timer;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.SubsystemMode;
import org.littletonrobotics.junction.Logger;

public class Deployer {

    private DeployerIO deployerIO;

    private DeployerIOInputsAutoLogged inputs = new DeployerIOInputsAutoLogged();

    private double requestedPos;

    private Timer deployTimer = new Timer();

    private Timer deployPosSetTimer = new Timer();

    private DeployerState deployerState = DeployerState.DISABLED;

    public enum DeployerState {

        DISABLED, FIRST_EXTEND, EXTEND, SMOOSH
    }

    public Deployer(DeployerIO deployerIO) {
        this.deployerIO = deployerIO;
    }

    public void inputsPeriodic() {
        deployerIO.updateInputs(inputs);
        Logger.processInputs("Intake/Deployer", inputs);
    }

    // Called at end of command processing in intake
    public void outputsPeriodic() {
        Logger.recordOutput("Intake/Deployer/isSmooshed", isSmooshed());
        Logger.recordOutput("Intake/Deployer/isExtended", isExtended());
        Logger.recordOutput("Intake/Deployer/isStowed", isStowed());
    }

    public void setDeployerState(DeployerState state) {
        deployerState = state;
        if (deployerState != DeployerState.FIRST_EXTEND) {
            deployTimer.stop();
            deployTimer.reset();
            deployPosSetTimer.stop();
            deployPosSetTimer.reset();
        }
        switch(deployerState) {
            case DISABLED ->
                {
                    break;
                }
            case FIRST_EXTEND ->
                {
                    deployerIO.setVoltage(Constants.Deployer.deployVoltage);
                    deployTimer.start();
                    if (deployTimer.hasElapsed(Constants.Deployer.deploySec) && !deployPosSetTimer.isRunning()) {
                        deployerIO.seedPosition(Constants.Deployer.pressedIntoBumperDeg);
                        deployPosSetTimer.start();
                    }
                    if (deployPosSetTimer.hasElapsed(Constants.Deployer.deployPosSetSec)) {
                        // maintain chain tension until position set has taken effect
                        deployerIO.setVoltage(0);
                        // Setting state to extend REQUIRED to make initial deploy logic work.
                        // Intake.java always sets to "first extend" state initially and overriding to "extend"
                        // allows isExtended() method to return true based on deployer position.
                        // This allows the intake call to isExtended() to return true which will change
                        // the requested state to extend and deployer will work normally from there.
                        deployerState = DeployerState.EXTEND;
                    }
                }
            case EXTEND ->
                {
                    requestedPos = Constants.Deployer.extendDeg;
                    if (isExtended()) {
                        // drop to bumper in coast mode to avoid stalling motor
                        deployerIO.setVoltage(0);
                    } else {
                        deployerIO.setPosition(requestedPos);
                    }
                }
            case SMOOSH ->
                {
                    requestedPos = Constants.Deployer.smooshDeg;
                    deployerIO.setPosition(requestedPos);
                }
        }
        Logger.recordOutput("Intake/Deployer/state", deployerState);
        Logger.recordOutput("Intake/Deployer/requestedPos", requestedPos);
    }

    public boolean isStowed() {
        if (Constants.deployerMode == SubsystemMode.DISABLED) {
            return true;
        } else if (deployerState == DeployerState.DISABLED || deployerState == DeployerState.FIRST_EXTEND) {
            return false;
        } else {
            return inputs.angleDeg <= Constants.Deployer.retractDeg + Constants.Deployer.tolerance;
        }
    }

    public boolean isExtended() {
        if (Constants.deployerMode == SubsystemMode.DISABLED) {
            return true;
        } else if (deployerState == DeployerState.DISABLED || deployerState == DeployerState.FIRST_EXTEND) {
            return false;
        } else {
            return inputs.angleDeg >= Constants.Deployer.extendDeg - Constants.Deployer.tolerance;
        }
    }

    public boolean isSmooshed() {
        if (Constants.deployerMode == SubsystemMode.DISABLED) {
            return true;
        } else if (deployerState == DeployerState.DISABLED || deployerState == DeployerState.FIRST_EXTEND) {
            return false;
        } else {
            return inputs.angleDeg >= Constants.Deployer.smooshDeg - Constants.Deployer.tolerance && inputs.angleDeg <= Constants.Deployer.smooshDeg + Constants.Deployer.tolerance;
        }
    }

    public double getAngle() {
        return inputs.angleDeg;
    }

    public void setBrakeMode(Boolean mode) {
        deployerIO.setBrakeMode(mode);
    }
}
