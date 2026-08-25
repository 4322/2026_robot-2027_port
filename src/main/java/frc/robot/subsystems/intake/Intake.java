package frc.robot.subsystems.intake;

import org.wpilib.driverstation.DriverStation;
import org.wpilib.command2.SubsystemBase;
import frc.robot.subsystems.intake.deployer.Deployer;
import frc.robot.subsystems.intake.deployer.Deployer.DeployerState;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.rollers.Rollers.RollersState;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {

    private final Deployer deployer;

    private final Rollers rollers;

    private IntakeState state = IntakeState.STARING_CONFIG;

    private IntakeState prevState = IntakeState.STARING_CONFIG;

    private boolean hasExtended = false;

    public Intake(Deployer deployer, Rollers rollers) {
        this.deployer = deployer;
        this.rollers = rollers;
    }

    public enum IntakeState {

        STARING_CONFIG,
        DEPLOY,
        IDLE,
        EJECT,
        SMOOSH,
        INTAKING
    }

    @Override
    public void periodic() {
        rollers.inputsPeriodic();
        deployer.inputsPeriodic();
    }

    public void periodicOutputs() {
        if (org.wpilib.driverstation.RobotState.isDisabled() && state != IntakeState.STARING_CONFIG) {
            setState(IntakeState.IDLE);
        }
        switch(state) {
            case STARING_CONFIG ->
                {
                    deployer.setDeployerState(DeployerState.DISABLED);
                    rollers.setState(RollersState.DISABLED);
                }
            case DEPLOY ->
                {
                    deployer.setDeployerState(DeployerState.FIRST_EXTEND);
                    rollers.setState(RollersState.DEPLOY);
                    if (deployer.isExtended()) {
                        hasExtended = true;
                        state = prevState;
                        rollers.setState(RollersState.IDLE);
                    }
                }
            case IDLE ->
                {
                    deployer.setDeployerState(DeployerState.EXTEND);
                    rollers.setState(RollersState.IDLE);
                }
            case INTAKING ->
                {
                    deployer.setDeployerState(DeployerState.EXTEND);
                    rollers.setState(RollersState.INTAKE);
                }
            case EJECT ->
                {
                    deployer.setDeployerState(DeployerState.EXTEND);
                    rollers.setState(RollersState.EJECT);
                }
            case SMOOSH ->
                {
                    deployer.setDeployerState(DeployerState.SMOOSH);
                    rollers.setState(RollersState.SMOOSH);
                }
        }
        deployer.outputsPeriodic();
        rollers.outputsPeriodic();
        Logger.recordOutput("Intake/CurrentState", state);
        Logger.recordOutput("Intake/hasExtended", hasExtended);
    }

    public IntakeState getState() {
        return state;
    }

    public IntakeState getPrevState() {
        return prevState;
    }

    public void setState(IntakeState state) {
        Logger.recordOutput("Intake/RequestedState", state);
        if (hasExtended) {
            prevState = this.state;
            this.state = state;
        } else {
            prevState = state;
            this.state = IntakeState.DEPLOY;
        }
    }

    public boolean isExtended() {
        return deployer.isExtended();
    }

    public boolean isSmooshed() {
        return deployer.isSmooshed();
    }

    // Has extended from deployment; hasn't gotten stuck in net
    public boolean hasExtended() {
        return hasExtended;
    }
}
