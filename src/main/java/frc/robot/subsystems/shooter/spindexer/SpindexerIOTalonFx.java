package frc.robot.subsystems.shooter.spindexer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStationErrors;

import frc.robot.constants.Constants;
import frc.robot.util.AverageStat;

public class SpindexerIOTalonFx implements SpindexerIO {
  private TalonFX leaderMotor;
  private TalonFX followerMotor;
  private double lastRequestedVelocity = -1;
  private double lastRequestedVoltage = -1;

  private TalonFXConfiguration config = new TalonFXConfiguration();
  private VelocityVoltage leaderVelocityRequest = new VelocityVoltage(0).withSlot(0);
  private VoltageOut leaderVoltageRequest = new VoltageOut(0);

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

  private final Debouncer leaderMotorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer followerMotorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  private AverageStat supplyAmpsAvgStat = new AverageStat(50);

  public SpindexerIOTalonFx() {
    leaderMotor = new TalonFX(Constants.Spindexer.leaderMotorId, Constants.Systemcore.bus0);
    followerMotor = new TalonFX(Constants.Spindexer.followerMotorId, Constants.Systemcore.bus0);

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = Constants.Spindexer.statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Spindexer.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerLimit = Constants.Spindexer.supplyCurrentLowerLimit;
    config.CurrentLimits.SupplyCurrentLowerTime = Constants.Spindexer.supplyCurrentLowerTime;

    config.MotorOutput.Inverted = Constants.Spindexer.leaderMotorInvert;
    config.MotorOutput.NeutralMode = Constants.Spindexer.neutralMode;

    config.Slot0.kS = Constants.Spindexer.kS;
    config.Slot0.kV = Constants.Spindexer.kV;
    config.Slot0.kP = Constants.Spindexer.kP;
    config.Slot0.kI = Constants.Spindexer.kI;
    config.Slot0.kD = Constants.Spindexer.kD;

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;

    config.Feedback.SensorToMechanismRatio = Constants.Spindexer.motorToMechanismRatio;

    StatusCode leaderConfigStatus = leaderMotor.getConfigurator().apply(config);
    StatusCode followerConfigStatus = followerMotor.getConfigurator().apply(config);
    StatusCode followerMotorSetStatus =
        followerMotor.setControl(
            new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Aligned));

    if (leaderConfigStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon "
              + leaderMotor.getDeviceID()
              + " error (Spindexer Leader): "
              + leaderConfigStatus.getDescription(),
          false);
    }
    if (followerConfigStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " error (Spindexer Follower): "
              + followerConfigStatus.getDescription(),
          false);
    }
    if (followerMotorSetStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " set follower error (Spindexer Follower): "
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
  public void updateInputs(SpindexerIOInputs inputs) {
    var leadMotorStatus =
        BaseStatusSignal.refreshAll(
            leaderVelocity,
            leaderAppliedVolts,
            leaderStatorCurrent,
            leaderSupplyCurrent,
            leaderTemp);

    var followMotorStatus =
        BaseStatusSignal.refreshAll(
            followerVelocity,
            followerAppliedVolts,
            followerStatorCurrent,
            followerSupplyCurrent,
            followerTemp);

    inputs.leaderMotorConnected = leaderMotorConnectedDebounce.calculate(leadMotorStatus.isOK());
    inputs.leaderVoltage = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderMechanismRPS = leaderVelocity.getValueAsDouble();
    inputs.leaderSupplyAmps = leaderSupplyCurrent.getValueAsDouble();
    inputs.leaderStatorAmps = leaderStatorCurrent.getValueAsDouble();
    inputs.leaderMotorTempC = leaderTemp.getValueAsDouble();

    inputs.followerMotorConnected =
        followerMotorConnectedDebounce.calculate(followMotorStatus.isOK());
    inputs.followerVoltage = followerAppliedVolts.getValueAsDouble();
    inputs.followerMechanismRPS = followerVelocity.getValueAsDouble();
    inputs.followerSupplyAmps = followerSupplyCurrent.getValueAsDouble();
    inputs.followerStatorAmps = followerStatorCurrent.getValueAsDouble();
    inputs.followerMotorTempC = followerTemp.getValueAsDouble();

    supplyAmpsAvgStat.addSample(inputs.leaderSupplyAmps + inputs.followerSupplyAmps);
    inputs.supplyAmpsAvg = supplyAmpsAvgStat.getAverage();
  }

  @Override
  public void setTargetMechanismRotations(double velocity) {
    if (velocity != lastRequestedVelocity) {
      if (velocity == 0) {
        leaderMotor.stopMotor();
      } else {
        leaderMotor.setControl(leaderVelocityRequest.withVelocity(velocity).withEnableFOC(true));
      }
    }

    lastRequestedVelocity = velocity;
  }

  @Override
  public void setTargetVoltage(double voltage) {
    if (voltage != lastRequestedVoltage) {
      if (voltage == 0) {
        leaderMotor.stopMotor();
      } else {
        leaderMotor.setControl(leaderVoltageRequest.withOutput(voltage).withEnableFOC(true));
      }
    }
  }

  @Override
  public void stop() {
    lastRequestedVelocity = 0;
    leaderMotor.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    leaderMotor.configNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
    followerMotor.configNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
