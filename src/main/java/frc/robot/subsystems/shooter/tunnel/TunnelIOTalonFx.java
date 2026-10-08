package frc.robot.subsystems.shooter.tunnel;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.reduxrobotics.canandcolor.Canandcolor;
import com.reduxrobotics.canandcolor.CanandcolorSettings;

import org.wpilib.math.filter.Debouncer;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.wpilib.driverstation.DriverStationErrors;

import frc.robot.constants.Constants;
import frc.robot.util.AverageStat;

public class TunnelIOTalonFx implements TunnelIO {
  private TalonFX motor;
  private Canandcolor canandcolor;
  private CanandcolorSettings canandcolorConfig = new CanandcolorSettings();
  private double lastRequestedVelocity = -1;

  private TalonFXConfiguration config = new TalonFXConfiguration();
  private VelocityVoltage velocityRequest = new VelocityVoltage(0).withSlot(0);

  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;

  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  private AverageStat supplyAmpsAvgStat = new AverageStat(50);

  public TunnelIOTalonFx() {
    canandcolor = new Canandcolor(Constants.Tunnel.canandcolorID);
    
    canandcolorConfig.setColorFramePeriod(10); // Set color frame period to 10ms
      CanandcolorSettings canandcolorConfigStatus =
          canandcolor.setSettings(canandcolorConfig, 1.0, 5);
      if (!canandcolorConfigStatus.isEmpty()) {
        DriverStationErrors.reportError(
            "Canandcolor "
                + canandcolor.getAddress()
                + " error (Tunnel Sensor): "
                + canandcolorConfigStatus,
            false);
      }
    motor = new TalonFX(Constants.Tunnel.motorId, Constants.Systemcore.bus0);

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = false;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = false;
    config.CurrentLimits.StatorCurrentLimit = Constants.Tunnel.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Tunnel.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;

    config.MotorOutput.Inverted = Constants.Tunnel.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Tunnel.neutralMode;

    config.Slot0.kS = Constants.Tunnel.kS;
    config.Slot0.kV = Constants.Tunnel.kV;
    config.Slot0.kP = Constants.Tunnel.kP;
    config.Slot0.kI = Constants.Tunnel.kI;
    config.Slot0.kD = Constants.Tunnel.kD;

    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.HardwareLimitSwitch.ReverseLimitEnable = false;

    config.Feedback.SensorToMechanismRatio = Constants.Tunnel.motorToMechanismRatio;

    StatusCode configStatus = motor.getConfigurator().apply(config);

    if (configStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon " + motor.getDeviceID() + " error (Tunnel): " + configStatus.getDescription(),
          false);
    }

    velocity = motor.getVelocity();
    appliedVolts = motor.getMotorVoltage();
    statorCurrent = motor.getStatorCurrent();
    supplyCurrent = motor.getSupplyCurrent();
    temp = motor.getDeviceTemp();

    // Configure periodic frames
    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);
    ParentDevice.optimizeBusUtilizationForAll(motor);
  }

  @Override
  public void updateInputs(TunnelIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(velocity, appliedVolts, statorCurrent, supplyCurrent, temp);

    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());
    inputs.voltage = appliedVolts.getValueAsDouble();
    inputs.mechanismRPS = velocity.getValueAsDouble();
    inputs.supplyAmps = supplyCurrent.getValueAsDouble();
    inputs.statorAmps = statorCurrent.getValueAsDouble();
    inputs.motorTempC = temp.getValueAsDouble();

    inputs.fuelDetected = canandcolor.getProximity() < Constants.Tunnel.minFuelDetection;

    supplyAmpsAvgStat.addSample(Math.abs(inputs.supplyAmps));
    inputs.supplyAmpsAbsAvg = supplyAmpsAvgStat.getAverage();
  }

  @Override
  public void setTargetMechanismRotations(double velocity) {
    if (velocity != lastRequestedVelocity) {
      if (velocity == 0) {
        motor.stopMotor();
      } else {
        motor.setControl(velocityRequest.withVelocity(velocity).withEnableFOC(true));
      }
    }

    lastRequestedVelocity = velocity;
  }

  @Override
  public void stop() {
    lastRequestedVelocity = 0;
    motor.stopMotor();
  }

  @Override
  public void enableBrakeMode(boolean enable) {
    motor.configNeutralMode(enable ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }
}
