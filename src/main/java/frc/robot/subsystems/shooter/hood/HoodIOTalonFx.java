package frc.robot.subsystems.shooter.hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
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

public class HoodIOTalonFx implements HoodIO {
  private TalonFX hoodMotor;
  private TalonFXConfiguration config = new TalonFXConfiguration();

  private final StatusSignal<Angle> position;
  private final StatusSignal<AngularVelocity> velocity;
  private final StatusSignal<Voltage> appliedVolts;
  private final StatusSignal<Current> supplyCurrent;
  private final StatusSignal<Current> statorCurrent;
  private final StatusSignal<Temperature> temp;

  private final Debouncer motorConnectedDebounce =
      new Debouncer(0.5, Debouncer.DebounceType.FALLING);

  public HoodIOTalonFx() {
    hoodMotor = new TalonFX(Constants.Hood.motorId, Constants.SystemcoreBus.systemcore);

    config.CurrentLimits.StatorCurrentLimit = Constants.Hood.statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = Constants.Hood.supplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLowerTime = 0;

    config.MotorOutput.Inverted = Constants.Hood.motorInvert;
    config.MotorOutput.NeutralMode = Constants.Hood.neutralMode;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    config.Feedback.SensorToMechanismRatio =
        Constants.Hood.motorToEncoderGearRatio * Constants.Hood.encoderToHoodGearRatio;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
        Units.degreesToRotations(Constants.Hood.minPhysicalLimitDeg);
    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
        Units.degreesToRotations(Constants.Hood.maxPhysicalLimitDeg);
    config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;
    config.Slot0.GravityArmPositionOffset = 0;

    config.Slot0.kG = Constants.Hood.kG;
    config.Slot0.kV = Constants.Hood.kV;
    config.Slot0.kP = Constants.Hood.kP;
    config.Slot0.kI = Constants.Hood.kI;
    config.Slot0.kD = Constants.Hood.kD;

    config.MotionMagic.MotionMagicCruiseVelocity = Constants.Hood.motionMagicCruiseVelocity;
    config.MotionMagic.MotionMagicAcceleration = Constants.Hood.motionMagicAcceleration;
    StatusCode configStatus = hoodMotor.getConfigurator().apply(config);
    if (configStatus != StatusCode.OK) {
      DriverStationErrors.reportError(
          "Talon " + hoodMotor.getDeviceID() + " error (Hood): " + configStatus.getDescription(),
          false);
    }

    position = hoodMotor.getPosition();
    velocity = hoodMotor.getVelocity();
    appliedVolts = hoodMotor.getMotorVoltage();
    statorCurrent = hoodMotor.getStatorCurrent();
    supplyCurrent = hoodMotor.getSupplyCurrent();
    temp = hoodMotor.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        50.0, position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);
  }

  @Override
  public void updateInputs(HoodIOInputs inputs) {
    var motorStatus =
        BaseStatusSignal.refreshAll(
            position, velocity, appliedVolts, statorCurrent, supplyCurrent, temp);

    inputs.motorConnected = motorConnectedDebounce.calculate(motorStatus.isOK());
    inputs.hoodDegrees = Units.rotationsToDegrees(position.getValueAsDouble());
    inputs.hoodRPS = velocity.getValueAsDouble();
    inputs.supplyCurrentAmps = supplyCurrent.getValueAsDouble();
    inputs.statorCurrentAmps = statorCurrent.getValueAsDouble();
    inputs.tempCelsius = temp.getValueAsDouble();
    inputs.appliedVoltage = appliedVolts.getValueAsDouble();
  }

  @Override
  public void setHomed() {
    hoodMotor.setPosition(Units.degreesToRotations(Constants.Hood.homingDegrees));
  }

  @Override
  public void setVoltage(double voltage) {
    hoodMotor.setControl(
        new VoltageOut(voltage).withEnableFOC(true).withIgnoreSoftwareLimits(true));
  }

  @Override
  public void setBrakeMode(boolean mode) {
    hoodMotor.configNeutralMode(mode ? NeutralModeValue.Brake : NeutralModeValue.Coast);
  }

  @Override
  public void setAngle(double degs) {
    hoodMotor.setControl(
        new MotionMagicVoltage(Units.degreesToRotations(degs)).withEnableFOC(true).withSlot(0));
  }
}
