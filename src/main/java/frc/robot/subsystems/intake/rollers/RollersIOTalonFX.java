package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.wpilib.driverstation.DriverStationErrors;

import frc.robot.constants.Constants;

public class RollersIOTalonFX implements RollersIO {
  private TalonFX motor;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;

  private final Debouncer connectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  public RollersIOTalonFX(boolean isLeftRoller) {
    int motorId = isLeftRoller ? Constants.Rollers.leftMotorId : Constants.Rollers.rightMotorId;
    CANBus bus = isLeftRoller ? Constants.Systemcore.bus2 : Constants.Systemcore.bus3;
    InvertedValue invertedValue =
        isLeftRoller ? Constants.Rollers.leftMotorInvert : Constants.Rollers.rightMotorInvert;

    motor = new TalonFX(motorId, bus);

    motorConfigs.MotorOutput.NeutralMode = Constants.Rollers.neutralMode;

    motorConfigs.HardwareLimitSwitch.ForwardLimitEnable = false;
    motorConfigs.HardwareLimitSwitch.ReverseLimitEnable = false;
    motorConfigs.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    motorConfigs.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    motorConfigs.CurrentLimits.StatorCurrentLimit = Constants.Rollers.statorCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimit = Constants.Rollers.supplyCurrentLimit;
    motorConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfigs.CurrentLimits.SupplyCurrentLowerTime = 0;

    motorConfigs.MotorOutput.Inverted = invertedValue;
    motorConfigs.MotorOutput.NeutralMode = Constants.Rollers.neutralMode;
    StatusCode ConfigStatus = motor.getConfigurator().apply(motorConfigs);

    if (ConfigStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon "
              + motor.getDeviceID()
              + " config error (Rollers ): "
              + ConfigStatus.getDescription(),
          false);
    }

    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        velocity,
        appliedVolts,
        statorCurrent,
        supplyCurrent,
        temp);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            velocity,
            appliedVolts,
            statorCurrent,
            supplyCurrent,
            temp);

    inputs.Connected = connectedDebounce.calculate(motorStatus.isOK());
    inputs.Volts = appliedVolts.getValueAsDouble();
    inputs.SupplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.StatorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.TempCelcius = temp.getValueAsDouble();
    inputs.RotationsPerSec = velocity.getValueAsDouble();
  }

  @Override
  public void setVoltage(double volts) {
    motor.setControl(new VoltageOut(volts));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  @Override
  public TalonFX getTalonFX() {
    return motor;

  }
}