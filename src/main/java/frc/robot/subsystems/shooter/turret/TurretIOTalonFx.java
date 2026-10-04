package frc.robot.subsystems.shooter.turret;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import org.wpilib.math.filter.Debouncer;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Current;
import org.wpilib.units.measure.Temperature;
import org.wpilib.units.measure.Voltage;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.DriverStationErrors;

import frc.robot.constants.Constants;

public class TurretIOTalonFx implements TurretIO {
  private TalonFX turretMotor;
  private CANcoder CANcoderOne;
  private CANcoder CANcoderTwo;
  private TalonFXConfiguration config = new TalonFXConfiguration();
  private CANcoderConfiguration CANconfigOne = new CANcoderConfiguration();
  private CANcoderConfiguration CANconfigTwo = new CANcoderConfiguration();

  private final StatusSignal<Angle> position;
  private final StatusSignal<Angle> absolutePositionOne;
  private final StatusSignal<Angle> absolutePositionTwo;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;

  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer encoderOneConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);
  private final Debouncer encoderTwoConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  public TurretIOTalonFx() {
    turretMotor = new TalonFX(Constants.Turret.motorId, Constants.Systemcore.bus0);
    CANcoderOne = new CANcoder(Constants.Turret.CANCoderOneId, Constants.Systemcore.bus0);
    CANcoderTwo = new CANcoder(Constants.Turret.CANCoderTwoId, Constants.Systemcore.bus0);

    config.CurrentLimits.StatorCurrentLimit = Constants.Turret.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Turret.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;

    config.MotorOutput.Inverted = Constants.Turret.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Turret.neutralMode;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio = Constants.Turret.turretGearRatio;

    config.Slot0.kS = Constants.Turret.kS;
    config.Slot0.kV = Constants.Turret.kV;
    config.Slot0.kP = Constants.Turret.kP;
    config.Slot0.kI = Constants.Turret.kI;
    config.Slot0.kD = Constants.Turret.kD;

    config.HardwareLimitSwitch.ReverseLimitEnable = false;
    config.HardwareLimitSwitch.ForwardLimitEnable = false;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(Constants.Turret.minPhysicalLimitDeg);
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(Constants.Turret.maxPhysicalLimitDeg);
    CANconfigOne.MagnetSensor.MagnetOffset = Constants.Turret.CANCoderOneOffsetRot;
    CANconfigTwo.MagnetSensor.MagnetOffset = Constants.Turret.CANCoderTwoOffsetRot;
    CANconfigOne.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1; // range 0 to 1.0
    CANconfigTwo.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
    CANconfigOne.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    CANconfigTwo.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Turret.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = Constants.Turret.motionMagicAcceleration;

    StatusCode configStatus = turretMotor.getConfigurator().apply(config);
    StatusCode CANcoderOneStatus = CANcoderOne.getConfigurator().apply(CANconfigOne);
    StatusCode CANcoderTwoStatus = CANcoderTwo.getConfigurator().apply(CANconfigTwo);

    if (configStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon "
              + turretMotor.getDeviceID()
              + " error (Turret): "
              + configStatus.getDescription(),
          false);
    }
    if (CANcoderOneStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "CANCoderOne "
              + CANcoderOne.getDeviceID()
              + " error (Turret): "
              + CANcoderOneStatus.getDescription(),
          false);
    }
    if (CANcoderTwoStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "CANCoderTwo "
              + CANcoderTwo.getDeviceID()
              + " error (Turret): "
              + CANcoderTwoStatus.getDescription(),
          false);
    }

    try {
      // wait for encoder positions to be received
      Thread.sleep(100);
    } catch (InterruptedException e) {
    }

    position = turretMotor.getPosition();
    absolutePositionOne = CANcoderOne.getAbsolutePosition();
    absolutePositionTwo = CANcoderTwo.getAbsolutePosition();
    velocity = turretMotor.getVelocity();
    appliedVolts = turretMotor.getMotorVoltage();
    statorCurrent = turretMotor.getStatorCurrent();
    supplyCurrent = turretMotor.getSupplyCurrent();
    temp = turretMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0,
        position,
        absolutePositionOne,
        absolutePositionTwo,
        velocity,
        appliedVolts,
        statorCurrent,
        supplyCurrent,
        temp);
    ParentDevice.optimizeBusUtilizationForAll(turretMotor);
  }

  @Override
  public void updateInputs(TurretIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);
    var encoderOneStatus = BaseStatusSignal.refreshAll(absolutePositionOne);
    var encoderTwoStatus = BaseStatusSignal.refreshAll(absolutePositionTwo);

    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());
    inputs.encoderOneConnected = encoderOneConnectedDebounce.calculate(encoderOneStatus.isOK());
    inputs.encoderTwoConnected = encoderTwoConnectedDebounce.calculate(encoderTwoStatus.isOK());
    inputs.turretDegs = Units.rotationsToDegrees(position.getValueAsDouble());
    inputs.encoderOneRot = absolutePositionOne.getValueAsDouble();
    inputs.encoderTwoRot = absolutePositionTwo.getValueAsDouble();
    inputs.motorRPS = velocity.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
  }

  @Override
  public void setBrakeMode(boolean mode) {
    turretMotor.configNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setAngle(double degs, double ffRadPerSec) {
    turretMotor.setControl(
        new MotionMagicVoltage(Units.degreesToRotations(degs))
            .withEnableFOC(true)
            .withSlot(0)
            .withFeedForward(ffRadPerSec * Constants.Turret.rotCompensationkV));
  }

  @Override
  public void setPosition(double rot) {
    turretMotor.setPosition(rot);
  }
}
