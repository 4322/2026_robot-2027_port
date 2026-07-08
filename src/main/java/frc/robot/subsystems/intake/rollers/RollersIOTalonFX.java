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
import frc.robot.constants.Constants;

public class RollersIOTalonFX implements RollersIO {
  private TalonFX leaderMotor;
  private TalonFX followerMotor;

  private Double previousRequestedVoltage;

  private TalonFXConfiguration motorConfigs = new TalonFXConfiguration();

  private final StatusSignal<AngularVelocity> leaderVelocity;
  private final StatusSignal<Voltage> leaderAppliedVolts;
  private final StatusSignal<Current> leaderSupplyCurrent;
  private final StatusSignal<Current> leaderStatorCurrent;
  private final StatusSignal<Temperature> leaderTemp;

  private final StatusSignal<AngularVelocity> followerVelocity;
  private final StatusSignal<Voltage> followerAppliedVolts;
  private final StatusSignal<Current> followerSupplyCurrent;
  private final StatusSignal<Current> followerStatorCurrent;
  private final StatusSignal<Temperature> followerTemp;

  private final Debouncer leaderConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);
  private final Debouncer followerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.kFalling);

  public RollersIOTalonFX() {
    leaderMotor = new TalonFX(Constants.Rollers.leaderMotorId);
    followerMotor = new TalonFX(Constants.Rollers.followerMotorId);

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

    motorConfigs.MotorOutput.Inverted = Constants.Rollers.leaderMotorInvert;
    motorConfigs.MotorOutput.NeutralMode = Constants.Rollers.neutralMode;
    StatusCode leaderConfigStatus = leaderMotor.getConfigurator().apply(motorConfigs);
    StatusCode followerConfigStatus = followerMotor.getConfigurator().apply(motorConfigs);
    StatusCode followerMotorSetStatus =
        followerMotor.setControl(
            new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    if (leaderConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + leaderMotor.getDeviceID()
              + " config error (Rollers Leader): "
              + leaderConfigStatus.getDescription(),
          false);
    }
    if (followerConfigStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " config error (Rollers Follower): "
              + followerConfigStatus.getDescription(),
          false);
    }
    if (followerMotorSetStatus != StatusCode.OK) {
      DriverStation.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " set follower error (Rollers Follower): "
              + followerMotorSetStatus.getDescription(),
          false);
    }

    leaderVelocity = leaderMotor.getVelocity();
    leaderAppliedVolts = leaderMotor.getMotorVoltage();
    leaderStatorCurrent = leaderMotor.getStatorCurrent();
    leaderSupplyCurrent = leaderMotor.getSupplyCurrent();
    leaderTemp = leaderMotor.getDeviceTemp();

    followerVelocity = followerMotor.getVelocity();
    followerAppliedVolts = followerMotor.getMotorVoltage();
    followerStatorCurrent = followerMotor.getStatorCurrent();
    followerSupplyCurrent = followerMotor.getSupplyCurrent();
    followerTemp = followerMotor.getDeviceTemp();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        leaderVelocity,
        leaderAppliedVolts,
        leaderStatorCurrent,
        leaderSupplyCurrent,
        leaderTemp,
        followerVelocity,
        followerAppliedVolts,
        followerStatorCurrent,
        followerSupplyCurrent,
        followerTemp);
    ParentDevice.optimizeBusUtilizationForAll(leaderMotor, followerMotor);
  }

  @Override
  public void updateInputs(RollersIOInputs inputs) {
    var leaderMotorStatus =
        BaseStatusSignal.refreshAll(
            leaderVelocity,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent,
            leaderTemp);

    var followerMotorStatus =
        BaseStatusSignal.refreshAll(
            followerVelocity,
            followerAppliedVolts,
            followerStatorCurrent,
            followerSupplyCurrent,
            followerTemp);

    inputs.leaderConnected = leaderConnectedDebounce.calculate(leaderMotorStatus.isOK());
    inputs.leaderVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderSupplyCurrentAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.leaderStatorCurrentAmps = leaderStatorCurrent.getValueAsDouble();
    inputs.leaderTempCelcius = leaderTemp.getValueAsDouble();
    inputs.leaderRotationsPerSec = leaderVelocity.getValueAsDouble();

    inputs.followerConnected = followerConnectedDebounce.calculate(followerMotorStatus.isOK());
    inputs.followerVolts = followerAppliedVolts.getValueAsDouble();
    inputs.followerSupplyCurrentAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerStatorCurrentAmps = followerStatorCurrent.getValueAsDouble();
    inputs.followerTempCelcius = followerTemp.getValueAsDouble();
    inputs.followerRotationsPerSec = followerVelocity.getValueAsDouble();
  }

  @Override
  public void setVoltage(double voltage) {
    if ((previousRequestedVoltage == null) || (previousRequestedVoltage != voltage)) {
      previousRequestedVoltage = voltage;
      leaderMotor.setControl(new VoltageOut(voltage));
    }
  }

  @Override
  public void stopMotor() {
    leaderMotor.stopMotor();
    previousRequestedVoltage = null;
  }

  @Override
  public TalonFX getTalonFX() {
    return leaderMotor;
  }
}
