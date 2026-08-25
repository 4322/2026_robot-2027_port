package frc.robot.subsystems.shooter.hood;

import org.wpilib.driverstation.DriverStation;
import frc.robot.constants.Constants;

public class HoodIOSim implements HoodIO {

    private double positionDeg = 0;

    private double requestedPositionDeg = 0;

    @Override
    public void updateInputs(HoodIOInputs inputs) {
        inputs.motorConnected = true;
        simPos();
        inputs.hoodDegrees = positionDeg;
    }

    @Override
    public void setHomed() {
        this.positionDeg = 0;
        this.requestedPositionDeg = 0;
    }

    @Override
    public void setAngle(double requestedPosDeg) {
        this.requestedPositionDeg = requestedPosDeg;
    }

    private void simPos() {
        if (org.wpilib.driverstation.RobotState.isEnabled()) {
            if (positionDeg < requestedPositionDeg) {
                positionDeg += (requestedPositionDeg - positionDeg) * Constants.Sim.hoodRate;
            } else {
                positionDeg -= (positionDeg - requestedPositionDeg) * Constants.Sim.hoodRate;
            }
        }
    }
}
