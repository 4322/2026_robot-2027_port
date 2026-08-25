package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.hardware.TalonFX;
import org.wpilib.util.Color;
import org.littletonrobotics.junction.AutoLog;

public interface FlywheelIO {

    @AutoLog
    public static class FlywheelIOInputs {

        public boolean leaderMotorConnected = false;

        public boolean followerMotorConnected = false;

        public double leaderMechanismRPS = 0.0;

        public double followerMechanismRPS = 0.0;

        public double leaderAppliedVolts = 0.0;

        public double followerAppliedVolts = 0.0;

        public double leaderTempCelsius = 0.0;

        public double leaderSupplyAmps = 0.0;

        public double followerTempCelsius = 0.0;

        public double followerSupplyAmps = 0.0;

        public double leaderStatorAmps = 0.0;

        public double followerStatorAmps = 0.0;

        public double supplyAmpsAvg = 0.0;

        public boolean sensorConnected = false;

        public Color color = new Color(0, 0, 0);

        public double proximity = 0.0;

        public boolean fuelDetected = false;
    }

    public default void updateInputs(FlywheelIOInputs inputs) {
    }

    public default void stop() {
    }

    public default TalonFX getTalonFX() {
        return null;
    }

    public default void setTargetMechanismRPS(double speedMechanismRotations) {
    }
}
