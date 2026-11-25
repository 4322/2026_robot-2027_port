package frc.robot.util.ClockworkLib;

import com.ctre.phoenix6.hardware.TalonFX;
import org.littletonrobotics.junction.AutoLog;

/**
 * Generic motor IO contract shared by all single-motor subsystems.
 *
 * <p>Implementations provide telemetry through {@link GenericMotorIOInputs} and accept
 * voltage/position commands. AdvantageKit logs the inputs via the nested {@code @AutoLog}
 * class so every motor subsystem exposes a consistent field set.
 */
public interface GenericMotorIO {
  /** Standard telemetry fields populated by {@link #updateInputs(GenericMotorIOInputs)}. */
  @AutoLog
  class GenericMotorIOInputs {
    /** Whether the motor controller is responding on the CAN bus. */
    public boolean connected = false;
    /** Stator current in amps. */
    public double statorCurrentAmps = 0.0;
    /** Supply current in amps. */
    public double supplyCurrentAmps = 0.0;
    /** Device temperature in Celsius. */
    public double tempCelcius = 0.0;
    /** Mechanism velocity in rotations per second. */
    public double velocityRPS = 0.0;
    /** Mechanism position in degrees. */
    public double positionDeg = 0.0;
    /** Applied motor voltage. */
    public double appliedVolts = 0.0;
  }

  /** Refreshes {@code inputs} with the latest motor telemetry. */
  default void updateInputs(GenericMotorIOInputs inputs) {}

  /** Commands the motor to the given voltage. */
  default void setVoltage(double volts) {}

  /** Commands a position setpoint in degrees (position-controlled mechanisms). */
  default void setPosition(double deg) {}

  /** Toggles brake/coast neutral mode. */
  default void setBrakeMode(boolean enable) {}

  /** Stops the motor. */
  default void stop() {}

  /** Returns the underlying TalonFX, or {@code null} for sim/non-Talon implementations. */
  default TalonFX getTalonFX() {
    return null;
  }
}
