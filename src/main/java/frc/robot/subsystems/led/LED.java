package frc.robot.subsystems.led;

import org.wpilib.driverstation.DriverStation;
import org.wpilib.command2.SubsystemBase;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.shooter.areaManager.AreaManager;
import frc.robot.util.HubShiftUtil;
import org.littletonrobotics.junction.Logger;

public class LED extends SubsystemBase {

    private LEDState state = LEDState.DISABLED;

    private boolean climberDeployed = false;

    private boolean autoFuelPickup = false;

    private boolean turretUnwinding = false;

    private LEDIO io;

    private Drive drive;

    public enum LEDState {

        DISABLED,
        CLIMBER_DEPLOYED,
        AUTO_FUEL_PICKUP,
        TURRET_UNWINDING,
        SHOOTING_AREA_ACTIVE,
        SHOOTING_AREA_INACTIVE,
        NON_SHOOTING_AREA
    }

    public enum AnimationType {

        COLOR_FLOW,
        FIRE,
        LARSON,
        RAINBOW,
        SINGLE_FADE,
        SOLID_COLOR,
        STROBE,
        TWINKLE,
        RGB_FADE
    }

    public LED(LEDIO io, Drive drive) {
        this.io = io;
        this.drive = drive;
    }

    @Override
    public void periodic() {
        Logger.recordOutput("LED/State", state.toString());
        if (org.wpilib.driverstation.RobotState.isDisabled()) {
            setLEDState(LEDState.DISABLED);
        } else if (climberDeployed) {
            setLEDState(LEDState.CLIMBER_DEPLOYED);
        } else if (autoFuelPickup) {
            setLEDState(LEDState.AUTO_FUEL_PICKUP);
        } else if (turretUnwinding) {
            setLEDState(LEDState.TURRET_UNWINDING);
        } else if (AreaManager.isShootingArea(drive.getRobotPose())) {
            if (HubShiftUtil.getShiftedShiftInfo().active()) {
                setLEDState(LEDState.SHOOTING_AREA_ACTIVE);
            } else {
                setLEDState(LEDState.SHOOTING_AREA_INACTIVE);
            }
        } else {
            setLEDState(LEDState.NON_SHOOTING_AREA);
        }
    }

    private void setLEDState(LEDState newState) {
        if (state != newState) {
            state = newState;
            io.clearLEDs();
            switch(state) {
                case DISABLED ->
                    {
                        io.setLEDs(AnimationType.RAINBOW, 0);
                    }
                case CLIMBER_DEPLOYED ->
                    {
                    }
                case AUTO_FUEL_PICKUP ->
                    {
                        io.setLEDs(AnimationType.STROBE, 0, 0, 0, 255);
                    }
                case TURRET_UNWINDING ->
                    {
                        io.setLEDs(AnimationType.LARSON, 0, 255, 0, 255);
                    }
                case SHOOTING_AREA_ACTIVE ->
                    {
                        io.setLEDs(AnimationType.COLOR_FLOW, 0, 0, 255, 0);
                    }
                case SHOOTING_AREA_INACTIVE ->
                    {
                        io.setLEDs(AnimationType.COLOR_FLOW, 0, 255, 255, 0);
                    }
                case NON_SHOOTING_AREA ->
                    {
                        io.setLEDs(AnimationType.COLOR_FLOW, 0, 255, 0, 0);
                    }
            }
        }
    }

    public void requestClimberDeployed(boolean value) {
        climberDeployed = value;
    }

    public void requestAutoFuelPickup(boolean value) {
        autoFuelPickup = value;
    }

    public void requestTurretUnwinding(boolean value) {
        turretUnwinding = value;
    }
}
