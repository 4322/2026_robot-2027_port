package frc.robot.util.ClockworkLib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;

/**
 * Reusable TalonFX implementation of {@link GenericMotorIO}.
 *
 * <p>Owns the TalonFX, applies configuration, registers status signals at the requested
 * update frequency, and fills {@link GenericMotorIO.GenericMotorIOInputs} on
 * {@link #updateInputs(GenericMotorIO.GenericMotorIOInputs)}.
 */
public class TalonFXIO implements GenericMotorIO {
  protected final TalonFX motor;
  protected final TalonFXConfiguration config;

  protected final StatusSignal<Angle> position;
  protected final StatusSignal<AngularVelocity> velocity;
  protected final StatusSignal<Voltage> appliedVolts;
  protected final StatusSignal<Current> supplyCurrent;
  protected final StatusSignal<Current> statorCurrent;
  protected final StatusSignal<Temperature> temperature;

  private final Debouncer connectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  /**
   * Creates and configures a TalonFX motor handle.
   *
   * @param motorId CAN id of the motor.
   * @param canBus CAN bus the motor is on.
   * @param config configuration to apply.
   * @param updateFrequencyHz update frequency for status signals.
   */
  protected TalonFXIO(
      int motorId,
      CANBus canBus,
      TalonFXConfiguration config,
      double updateFrequencyHz) {

    this.motor = new TalonFX(motorId, canBus);
    this.config = config;

    StatusCode status = motor.getConfigurator().apply(config);
    if (status != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon " + motor.getDeviceID() + " configuration error: " + status.getDescription(),
          false);
    }

    position = motor.getPosition();
    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    supplyCurrent = motor.getSupplyCurrent();
    statorCurrent = motor.getStatorCurrent();
    temperature = motor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        updateFrequencyHz,
        position,
        velocity,
        appliedVolts,
        supplyCurrent,
        statorCurrent,
        temperature);

    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(GenericMotorIOInputs inputs) {
    var status =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, supplyCurrent, statorCurrent, temperature);

    inputs.connected = connectedDebounce.calculate(status.isOK());
    inputs.positionDeg = Units.rotationsToDegrees(position.getValueAsDouble());
    inputs.velocityRPS = velocity.getValueAsDouble();
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.tempCelcius = temperature.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts).withEnableFOC(true));
  }

  @Override
  public void setBrakeMode(boolean enable) {
    motor.setNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public TalonFX getTalonFX() {
    return motor;
  }

  /** Exposes the underlying configuration for subclasses. */
  protected TalonFXConfiguration getConfig() {
    return config;
  }
}
