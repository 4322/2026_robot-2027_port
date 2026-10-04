package frc.robot.subsystems.shooter.flywheel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.reduxrobotics.canandcolor.Canandcolor;
import com.reduxrobotics.canandcolor.CanandcolorSettings;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.wpilib.driverstation.DriverStationErrors;
import org.wpilib.util.Color;
import frc.robot.constants.Constants;
import frc.robot.util.AverageStat;

public class FlywheelIOTalonFx implements FlywheelIO {

  private TalonFX leaderMotor;
  private TalonFX followerMotor;
  private Canandcolor canandcolor;
  private CanandcolorSettings canandcolorConfig = new CanandcolorSettings();
  private double lastRequestedVelocity = -1;

  private TalonFXConfiguration config = new TalonFXConfiguration();
  private VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

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
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer followerConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  private AverageStat supplyAmpsAvgStat = new AverageStat(50);

  public FlywheelIOTalonFx() {
    leaderMotor = new TalonFX(Constants.Flywheel.leaderMotorId, Constants.Systemcore.bus4);
    followerMotor = new TalonFX(Constants.Flywheel.followerMotorId, Constants.Systemcore.bus4);

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = Constants.Flywheel.statorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Flywheel.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;

    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Flywheel.motorToMechanismRatio;

    config.MotorOutput.Inverted = Constants.Flywheel.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Flywheel.neutralMode;

    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;

    config.Slot0.kS = Constants.Flywheel.kS;
    config.Slot0.kV = Constants.Flywheel.kV;
    config.Slot0.kP = Constants.Flywheel.kP;
    config.Slot0.kI = Constants.Flywheel.kI;
    config.Slot0.kD = Constants.Flywheel.kD;

    // Used for idle RPS to avoid wild PID oscillations when requesting slow RPS
    config.Slot1.kS = Constants.Flywheel.kS;
    config.Slot1.kV = Constants.Flywheel.kV;
    config.Slot1.kP = 0;
    config.Slot1.kI = 0;
    config.Slot1.kD = 0;

    StatusCode leaderConfigStatus = leaderMotor.getConfigurator().apply(config);
    StatusCode followerConfigStatus = leaderMotor.getConfigurator().apply(config);
    StatusCode followerMotorSetStatus =
        followerMotor.setControl(
            new Follower(leaderMotor.getDeviceID(), MotorAlignmentValue.Opposed));

    canandcolorConfig.setColorFramePeriod(10); // Set color frame period to 10ms

    if (Constants.Flywheel.canAndColorEnabled) {
      canandcolor = new Canandcolor(Constants.Flywheel.canandcolorId);
      CanandcolorSettings canandcolorConfigStatus =
          canandcolor.setSettings(canandcolorConfig, 1.0, 5);
      if (!canandcolorConfigStatus.isEmpty()) {
        DriverStationErrors.reportError(
            "Canandcolor "
                + canandcolor.getAddress()
                + " error (Flywheel Sensor): "
                + canandcolorConfigStatus,
            false);
      }
    }

    if (leaderConfigStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon "
              + leaderMotor.getDeviceID()
              + " config error (Flywheel Leader): "
              + leaderConfigStatus.getDescription(),
          false);
    }
    if (followerConfigStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " config error (Flywheel Follower): "
              + followerConfigStatus.getDescription(),
          false);
    }
    if (followerMotorSetStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon "
              + followerMotor.getDeviceID()
              + " set follower error (Flywheel Follower): "
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
  public void updateInputs(FlywheelIOInputs inputs) {
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

    inputs.leaderMotorConnected = leaderConnectedDebounce.calculate(leaderMotorStatus.isOK());
    inputs.leaderMechanismRPS = leaderVelocity.getValueAsDouble();
    inputs.leaderAppliedVolts = leaderAppliedVolts.getValueAsDouble();
    inputs.leaderTempCelsius = leaderTemp.getValueAsDouble();
    inputs.leaderStatorAmps = leaderStatorCurrent.getValueAsDouble();
    inputs.leaderSupplyAmps = leaderSupplyCurrent.getValueAsDouble();

    inputs.followerMotorConnected = followerConnectedDebounce.calculate(followerMotorStatus.isOK());
    inputs.followerMechanismRPS = followerVelocity.getValueAsDouble();
    inputs.followerAppliedVolts = followerAppliedVolts.getValueAsDouble();
    inputs.followerTempCelsius = followerTemp.getValueAsDouble();
    inputs.followerStatorAmps = followerStatorCurrent.getValueAsDouble();
    inputs.followerSupplyAmps = followerSupplyCurrent.getValueAsDouble();

    supplyAmpsAvgStat.addSample(inputs.leaderSupplyAmps + inputs.followerSupplyAmps);
    inputs.supplyAmpsAvg = supplyAmpsAvgStat.getAverage();

    if (Constants.Flywheel.canAndColorEnabled) {
      inputs.color = new Color(canandcolor.getRed(), canandcolor.getGreen(), canandcolor.getBlue());
      inputs.proximity = canandcolor.getProximity();

      inputs.proximity = canandcolor.getProximity();
      inputs.sensorConnected = canandcolor.isConnected();

      inputs.fuelDetected = inputs.proximity < Constants.Flywheel.minFuelDetectionProximity;
    }
  }

  @Override
  public void setTargetMechanismRPS(double mechanismRPS) {
    if (mechanismRPS != lastRequestedVelocity) {
      if (mechanismRPS == 0) {
        leaderMotor.stopMotor();
      } else {
        if (mechanismRPS == Constants.Flywheel.idleRPS) {
          leaderMotor.setControl(
              velocityRequest.withVelocity(mechanismRPS).withEnableFOC(true).withSlot(1));
        } else {
          leaderMotor.setControl(
              velocityRequest.withVelocity(mechanismRPS).withEnableFOC(true).withSlot(0));
        }
      }
    }

    lastRequestedVelocity = mechanismRPS;
  }

  @Override
  public void stop() {
    leaderMotor.stopMotor();
    lastRequestedVelocity = 0;
  }

  public TalonFX getTalonFX() {
    return leaderMotor;
  }
}
