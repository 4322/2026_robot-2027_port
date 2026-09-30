package frc.robot.subsystems.intake.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStationErrors;

import frc.robot.constants.Constants;

public class RollersIOTalonFX implements RollersIO {
  private TalonFX leftMotor;
  private TalonFX rightMotor;

  private Double previousRequestedVoltage;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  private final StatusSignal<AngularVelocity> leftVelocity;
  private final StatusSignal<Voltage> leftAppliedVolts;
  private final StatusSignal<Current> leftSupplyCurrent;
  private final StatusSignal<Current> leftStatorCurrent;
  private final StatusSignal<Temperature> leftTemp;

  private final StatusSignal<AngularVelocity> rightVelocity;
  private final StatusSignal<Voltage> rightAppliedVolts;
  private final StatusSignal<Current> rightSupplyCurrent;
  private final StatusSignal<Current> rightStatorCurrent;
  private final StatusSignal<Temperature> rightTemp;

  private final Debouncer leftConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer rightConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  public RollersIOTalonFX() {
    leftMotor = new TalonFX(Constants.Rollers.leftMotorId, Constants.Systemcore.bus2);
    rightMotor = new TalonFX(Constants.Rollers.rightMotorId, Constants.Systemcore.bus3);

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

    motorConfigs.MotorOutput.Inverted = Constants.Rollers.leftMotorInvert;
    motorConfigs.MotorOutput.NeutralMode = Constants.Rollers.neutralMode;
    StatusCode leftConfigStatus = leftMotor.getConfigurator().apply(motorConfigs);
    StatusCode rightConfigStatus = rightMotor.getConfigurator().apply(motorConfigs);
    StatusCode rightMotorSetStatus =
        rightMotor.setControl(
            new Follower(leftMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    if (leftConfigStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon "
              + leftMotor.getDeviceID()
              + " config error (Rollers Left): "
              + leftConfigStatus.getDescription(),
          false);
    }
    if (rightConfigStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon "
              + rightMotor.getDeviceID()
              + " config error (Rollers Right): "
              + rightConfigStatus.getDescription(),
          false);
    }
    if (rightMotorSetStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon "
              + rightMotor.getDeviceID()
              + " set right error (Rollers Right): "
              + rightMotorSetStatus.getDescription(),
          false);
    }

    leftVelocity = leftMotor.getVelocity();
    leftAppliedVolts = leftMotor.getMotorVoltage();
    leftStatorCurrent = leftMotor.getStatorCurrent();
    leftSupplyCurrent = leftMotor.getSupplyCurrent();
    leftTemp = leftMotor.getDeviceTemp();

    rightVelocity = rightMotor.getVelocity();
    rightAppliedVolts = rightMotor.getMotorVoltage();
    rightStatorCurrent = rightMotor.getStatorCurrent();
    rightSupplyCurrent = rightMotor.getSupplyCurrent();
    rightTemp = rightMotor.getDeviceTemp();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leftVelocity,
        leftAppliedVolts,
        leftStatorCurrent,
        leftSupplyCurrent,
        leftTemp,
        rightVelocity,
        rightAppliedVolts,
        rightStatorCurrent,
        rightSupplyCurrent,
        rightTemp);
    ParentDevice.optimizeBusUtilizationForAll(leftMotor, rightMotor);
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    var leftMotorStatus =
        BaseStatusSignal.refreshAll(
            leftVelocity,
            leftAppliedVolts,
            leftStatorCurrent,
            leftSupplyCurrent,
            leftTemp);

    var rightMotorStatus =
        BaseStatusSignal.refreshAll(
            rightVelocity,
            rightAppliedVolts,
            rightStatorCurrent,
            rightSupplyCurrent,
            rightTemp);

    inputs.leftConnected = leftConnectedDebounce.calculate(leftMotorStatus.isOK());
    inputs.leftVolts = leftAppliedVolts.getValueAsDouble();
    inputs.leftSupplyCurrentAmps = leftSupplyCurrent.getValueAsDouble();
    inputs.leftStatorCurrentAmps = leftStatorCurrent.getValueAsDouble();
    inputs.leftTempCelcius = leftTemp.getValueAsDouble();
    inputs.leftRotationsPerSec = leftVelocity.getValueAsDouble();

    inputs.rightConnected = rightConnectedDebounce.calculate(rightMotorStatus.isOK());
    inputs.rightVolts = rightAppliedVolts.getValueAsDouble();
    inputs.rightSupplyCurrentAmps = rightSupplyCurrent.getValueAsDouble();
    inputs.rightStatorCurrentAmps = rightStatorCurrent.getValueAsDouble();
    inputs.rightTempCelcius = rightTemp.getValueAsDouble();
    inputs.rightRotationsPerSec = rightVelocity.getValueAsDouble();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftMotor.setControl(new VoltageOut(leftVolts));
    rightMotor.setControl(new VoltageOut(rightVolts));
  }

  @Override
  public void stop() {
    leftMotor.stopMotor();
    rightMotor.stopMotor();
    previousRequestedVoltage = null;
  }

  @Override
  public TalonFX getLeftTalonFX() {
    return leftMotor;
  }

  @Override
  public TalonFX getRightTalonFX() {
    return rightMotor;
  }
}