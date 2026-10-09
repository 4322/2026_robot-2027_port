package frc.robot.constants;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StripTypeValue;
import org.wpilib.math.geometry.Rotation3d;
import org.wpilib.math.geometry.Transform3d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.geometry.Translation3d;
import org.wpilib.math.interpolation.InterpolatingDoubleTreeMap;
import org.wpilib.math.interpolation.InterpolatingTreeMap;
import org.wpilib.math.interpolation.InverseInterpolator;
import org.wpilib.math.util.Units;
import org.wpilib.units.measure.Time;
import org.wpilib.framework.RobotBase;
import org.wpilib.hardware.bus.CANPort;
import org.wpilib.util.Color;
import frc.robot.subsystems.shooter.FiringSolution;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection.ObjectDetectionType;
import java.util.ArrayList;

/**
 * This class defines the runtime mode used by AdvantageKit. The mode is always "real" when running
 * on a roboRIO. Change the value of "simMode" to switch between "sim" (physics sim) and "replay"
 * (log replay from a file).
 */
public final class Constants {
  public static final Mode currentMode = RobotBase.isReal() ? Mode.REAL : Mode.SIM;

  public static enum Mode {
    /** Running on a real robot. */
    REAL,

    /** Running a physics simulator. */
    SIM,

    /** Replaying from a log file. */
    REPLAY
  }

  public enum SubsystemMode {
    DISABLED,
    NORMAL,
    TUNING
  }

  public static SubsystemMode driveMode = SubsystemMode.NORMAL;
  public static final SubsystemMode flywheelMode = SubsystemMode.NORMAL;
  public static final SubsystemMode hoodMode = SubsystemMode.NORMAL;
  public static final SubsystemMode spindexerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode tunnelMode = SubsystemMode.NORMAL;
  public static SubsystemMode turretMode = SubsystemMode.NORMAL;
  public static final SubsystemMode deployerMode = SubsystemMode.NORMAL;
  public static final SubsystemMode rollerLeftMode = SubsystemMode.NORMAL;
  public static final SubsystemMode rollerRightMode = SubsystemMode.NORMAL;
  public static final SubsystemMode ledMode = SubsystemMode.DISABLED;
  public static final SubsystemMode visionGlobalPose = SubsystemMode.NORMAL;
  public static final SubsystemMode visionObjectDetection = SubsystemMode.DISABLED;
  public static final SubsystemMode firingManagerMode = SubsystemMode.DISABLED;
  public static final boolean turretLocked = false;
  public static final boolean frontRightCameraEnable = false;
  public static final boolean frontLeftCameraEnable = true;
  public static final boolean backRightCameraEnable = true;
  public static final boolean backLeftCameraEnable = true;
  public static final boolean enableCharacterizationAutos = false;
  public static final boolean tuningWithLoggableNumbers =
      (driveMode == SubsystemMode.TUNING
          || firingManagerMode == SubsystemMode.TUNING
          || hoodMode == SubsystemMode.TUNING
          || visionGlobalPose == SubsystemMode.TUNING);

  public static final double scoringDoubleToleranceTime = 0.5;
  public static final double passingDoubleToleranceTime = 0.25;

  { // set dependent operational modes
    if (turretLocked) {
      turretMode = SubsystemMode.DISABLED;
    }
    if (Constants.Drive.zeroTurnEncoders) {
      driveMode = SubsystemMode.TUNING;
    }
  }

  public static final boolean buzz = false;
  public static final String logPath = "/home/systemcore/logs";
  public static final long minFreeSpace = 1000000000; // 1 GB
  public static final int dioCoastButton = 0;
  public static final double coastButtonDelaySec = 10.0;
  public static final double brownoutVoltage = 5.75;
  public static final double brownoutRecoveryVoltage = 7; // TODO

  // only set this if loop cycles are significantly below 20 ms
  // to avoid starvation of critical processes
  public static final boolean realTimeCommandScheduler = false;

  public static final boolean debugZoneAreas = false;

  public static final boolean enableElasticOdometry = true;

  public static class Drive {
    public static final int gyroID = 0;
    public static final CANPort encoderCANPort = CANPort.CAN_S1;
    public static boolean zeroTurnEncoders = false; // for initial swerve homing only
    public static double driveSupplyCurrentLimit = 45; // don't pop main breaker
    public static double driveSupplyCurrentLowerLimit = 35;
    public static double driveSupplyCurrentLowerTime = 0.5;
    public static double turnSupplyCurrentLimit = 30;

    // Scales max speeds for X, Y, and omega respectively
    public static double maxLinearSpeedPercentShooting = 0.179;
    public static double maxAngularSpeedPercentShooting = 0.179;

    public static double maxLinearSpeedPercentPassing = 0.2626;
    public static double maxAngularSpeedPercentPassing = 0.2149;
  }

  public static class Spindexer {
    public static final int leaderMotorId = 4;
    public static final int followerMotorId = 6;

    public static final double supplyCurrentLimit = 60;
    public static final double supplyCurrentLowerLimit = 40;
    public static final double supplyCurrentLowerTime = 0.5; // fast start
    public static final double statorCurrentLimit = 120;
    public static final InvertedValue leaderMotorInvert = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double kS = 0.29;
    public static final double kV = 0.9;
    public static final double kP = 5.0; // TODO
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double stoppedMechanismRotationsPerSec = 0.1;
    public static final double stopTreshold = 0.75;

    public static final double motorToMechanismRatio = 36 / 12.0 * (90 / 35.0); // 10 inch wheel
    // Normally 7 RPS for shooting
    public static final double unjamRPS = -4.0;
    public static final double shootRPS = 9.0;
  }

  public static class Tunnel {
    public static final int motorId = 20;
    public static final double statorCurrentLimit = 80;
    public static final double supplyCurrentLimit = 40;
    public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;
    public static final double kS = 0.31;
    public static final double kV = 0.19;
    public static final double kP = 0.5;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double stoppedMechanismRotationsPerSec = 2;
    public static final double motorToMechanismRatio = 1.5; // 2 inch diameter
    public static final double minPercentVelocity = 0.95;
    // Normally 37 RPS for shooting
    public static final double unjamRPS = -25.0;
    public static final double shootRPS = 45;

    public static final int canandcolorID = 0; // TODO
    public static final double shootingTimeout = 0.5; // Amount of time that can pass without ball detected before declaring we are not shooting
    public static final double minFuelDetection = 0; // For canandcolor
    public static final double simShootingTime = 0.01; // Amount of time to shoot for before "empty", needs to be > 0
  }

  public static class Flywheel {
    public static final int leaderMotorId = 2;
    public static final int followerMotorId = 3;
    public static final double statorCurrentLimit = 120;
    public static final double supplyCurrentLimit = 40;
    public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;

    // max RPS for a burst is 75 due to drop in battery voltage
    public static final double kS = 0.32;
    public static final double kV = 0.123;
    public static final double kP = 0.75;
    public static final double kI = 0;
    public static final double kD = 0;

    public static final double motorToMechanismRatio = 1;
    public static final double largeToleranceRPS = 6.0;
    public static final double smallToleranceRPS = 4.0;
    public static final int idleRPS = 15;
    public static final int idleTimeout = 5;
  }

  public static class Turret {
    public static final int motorId = 22;
    public static final double kS = 0.26;
    public static final double kV = 0;
    public static final double kP = 150;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double rotCompensationkV = 1.0;

    public static final double motionMagicCruiseVelocity = 3.5;
    public static final double motionMagicAcceleration = 14.0;

    public static final double statorCurrentLimit = 60; // HACK set limits
    public static final double supplyCurrentLimit = 40; // set limits
    public static final InvertedValue motorInvert = InvertedValue.CounterClockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double smallToleranceDeg = 3.0;
    public static final double largeToleranceDeg = 6.0;
    public static final double goalToleranceLockedDeg = 2.0;
    public static final double CANCoderOneRatio = 90.0 / 10.0;
    public static final double CANCoderTwoRatio = 90.0 / 19.0;
    public static final double turretGearRatio = 27;
    public static final double minPhysicalLimitDeg = -180;
    public static final double maxPhysicalLimitDeg = 540;
    public static final double midPointPhysicalDeg =
        (minPhysicalLimitDeg + maxPhysicalLimitDeg) / 2.0;
    public static final double unwindToleranceDeg = 30.0;
    public static final double minUnwindLimitDeg = minPhysicalLimitDeg + unwindToleranceDeg;
    public static final double maxUnwindLimitDeg = maxPhysicalLimitDeg - unwindToleranceDeg;
    public static final int CANCoderOneId = 4;
    public static final int CANCoderTwoId = 1;
    public static final Translation2d originToTurret =
        new Translation2d(Units.inchesToMeters(-4.6111), Units.inchesToMeters(5.8889));

    public static final double unjamDeg = 90.0;

    // Encoder calibration procedure:
    // 1. Put turret in locked position and insert lock bolt (90 degrees)
    // 2. Set magnetic offsets to 0 on both encoders in Phoenix Tuner
    // 3. Set CANCoderOneOffsetRot = 0.25 - (PhoenixTuner AbsolutePosition)
    // 4. Set CANCoderTwoOffsetRot = 0.8157 - (PhoenixTuner AbsolutePosition)
    // 5. Deploy
    // 6. Verify correct turret degrees
    // 7. Remove lock bolt
    //    CANCoderOne = Encoder 9
    //    CANCoderTwo = Encoder 4.73
    public static final double CANCoderOneOffsetRot = 0.25 - 0.08;
    public static final double CANCoderTwoOffsetRot = 0.8157 - 0.3996;

    // Derivation of above values:
    // 290 degrees * 90/10 = encoder 1 should have rotated 7.25 rotations ->
    //   encoder 1 reads 0.25 in locked position
    // 290 degrees * 90/19 = encoder 2 should have rotated 3.815789 rotations ->
    //   encoder 2 reads 3341.0/4096.0 = 0.8157 in locked position
  }

  public static class Hood {
    public static final int motorId = 7;
    public static final int statorCurrentLimit = 60;
    public static final int supplyCurrentLimit = 15;
    public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final double encoderToHoodGearRatio = 164 / 11.0;
    public static final double motorToEncoderGearRatio = 5.0;
    public static final double safeAngleDeg = 0;
    public static final double homingDegrees =
        -0.35; // eliminate current draw and squeal when hood is down
    public static final double homingVoltage = -1.5;
    public static final double homingVelocityThresholdRPS = 0.01;
    public static final double minHomingSec = 0.150;
    public static final double smallToleranceDeg = 2.0; // shoot early, the hood will get there
    public static final double largeToleranceDeg = 3.0;
    public static final int idleTimeout = 0;
    public static final double minPhysicalLimitDeg = 0.0;
    public static final double maxPhysicalLimitDeg =
        37.0; // physical max 37.4 - don't overrun the stop

    public static final double kV = 0;
    public static final double kP = 3000; // motion magic prevents oscillations
    public static final double kD = 0;
    public static final double kI = 0;
    public static final double kG = 0.24;
    public static final double motionMagicCruiseVelocity = 1.3;
    public static final double motionMagicAcceleration = 5.0;
  }

  public class Rollers {
    public static final double voltageIntake = 8.0;
    public static final double voltageEject = -6;
    public static final double voltageDeploy = 0; // -1 to clear net, not good with extra top bar
    public static final double voltageIdle = 0;
    public static final double voltageSmoosh = 3;
    public static final int leftMotorId = 1;
    public static final int rightMotorId = 5;
    public static final double statorCurrentLimit = 100;
    public static final double supplyCurrentLimit = 30;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Coast;
    public static final InvertedValue leftMotorInvert = InvertedValue.CounterClockwise_Positive;
    public static final InvertedValue rightMotorInvert = InvertedValue.Clockwise_Positive;
  }

  public class Deployer {
    // 0 degrees is stowed postion
    // postive degrees when extending
    public static final double retractDeg = 7; // allow for net (starting config = 3.87)
    public static final double extendDeg = 122.0; // allow for backlash, 125.6 resting on bumper
    public static final double pressedIntoBumperDeg = 127.8;
    public static final double smooshDeg = 59.0;
    public static final double maxGravityDegrees = 125 - 180; // range is +/- 90 degrees
    public static final int motorId = 25;
    public static final double statorCurrentLimit = 120;
    public static final double supplyCurrentLimit = 40;
    public static final NeutralModeValue neutralMode = NeutralModeValue.Brake;
    public static final InvertedValue motorInvert = InvertedValue.Clockwise_Positive;
    public static final SensorDirectionValue sensorDirection =
        SensorDirectionValue.CounterClockwise_Positive;
    public static final double kP = 1000;
    public static final double kG = 0.65;
    public static final double kI = 0;
    public static final double kD = 0;
    public static final double motionMagicCruiseVelocity = 0.7;
    public static final double motionMagicAcceleration = 2.8;
    public static final int CANCoderID = 2;
    public static final double sensorToMechanismRatio = 3.0;
    public static final double RotorToSensorRatio = 12.0;
    public static final double tolerance = 3.0;
    public static final double SesnorOffsetRotations = 0.39; // retract pos > 0.02 to avoid wrapping
    public static final double deployVoltage = 12.0;
    public static final double deploySec = 0.9;
    public static final double deployPosSetSec = 0.1;
  }

  public static class FiringParameters {
    private final double flywheelRPS;
    private final double hoodAngleDeg;
    private final double timeOfFlightSec;
    private final double tunnelRPS;
    private final double indexerRPS;

    public FiringParameters(
        double flywheelRPS,
        double hoodAngleDeg,
        double timeOfFlightSec,
        double tunnelRPS,
        double indexerRPS) {
      this.flywheelRPS = flywheelRPS;
      this.hoodAngleDeg = hoodAngleDeg;
      this.timeOfFlightSec = timeOfFlightSec;
      this.tunnelRPS = tunnelRPS;
      this.indexerRPS = indexerRPS;
    }

    public double getFlywheelRPS() {
      return flywheelRPS;
    }

    public double getTunnelRPS() {
      return tunnelRPS;
    }

    public double getIndexerRPS() {
      return indexerRPS;
    }

    public double getHoodAngleDeg() {
      return hoodAngleDeg;
    }

    public double getTimeOfFlightSec() {
      return timeOfFlightSec;
    }

    public static FiringParameters interpolate(
        FiringParameters start, FiringParameters end, double howFar) {
      return new FiringParameters(
          start.flywheelRPS + (end.flywheelRPS - start.flywheelRPS) * howFar,
          start.hoodAngleDeg + (end.hoodAngleDeg - start.hoodAngleDeg) * howFar,
          start.timeOfFlightSec + (end.timeOfFlightSec - start.timeOfFlightSec) * howFar,
          start.tunnelRPS + (end.tunnelRPS - start.tunnelRPS) * howFar,
          start.indexerRPS + (end.indexerRPS - start.indexerRPS) * howFar);
    }
  }

  public static record ShotCalculatorParameters(
      double flywheelRPS,
      double hoodAngleDeg,
      double timeOfFlightSec,
      double tunnelRPS,
      double indexerRPS,
      double distanceMeters) {}

  public static class FiringManager {
    public static final double minTimeOfFlight = 0; // TODO
    public static final double maxTimeOfFlight = 4; // TODO

    public static final InterpolatingTreeMap<Double, FiringParameters> firingMapScoring =
        new InterpolatingTreeMap<Double, FiringParameters>(
            InverseInterpolator.forDouble(), FiringParameters::interpolate);
    public static final InterpolatingTreeMap<Double, FiringParameters> firingMapPassing =
        new InterpolatingTreeMap<Double, FiringParameters>(
            InverseInterpolator.forDouble(), FiringParameters::interpolate);

    // Reverse maps for velocity to distance lookup
    public static final InterpolatingDoubleTreeMap velocityToDistanceMapScoring =
        new InterpolatingDoubleTreeMap();
    public static final InterpolatingDoubleTreeMap velocityToDistanceMapPassing =
        new InterpolatingDoubleTreeMap();

    public static final double latencyCompensationScoring = 0.01;
    public static final double latencyCompensationPassing = 0.01;

    public static ArrayList<ShotCalculatorParameters> firingParametersListScoring =
        new ArrayList<ShotCalculatorParameters>();

    public static ArrayList<ShotCalculatorParameters> firingParametersListPassing =
        new ArrayList<ShotCalculatorParameters>();

    // Add entry to both maps
    public static void putFiringMapEntryScoring(double meters, FiringParameters params) {
      firingMapScoring.put(meters, params);
      double velocity = meters / params.getTimeOfFlightSec();
      velocityToDistanceMapScoring.put(velocity, meters);
      firingParametersListScoring.add(
          new ShotCalculatorParameters(
              params.getFlywheelRPS(),
              params.getHoodAngleDeg(),
              params.getTimeOfFlightSec(),
              params.getTunnelRPS(),
              params.getIndexerRPS(),
              meters));
    }

    public static void putFiringMapEntryPassing(double meters, FiringParameters params) {
      firingMapPassing.put(meters, params);
      double velocity = meters / params.getTimeOfFlightSec();
      velocityToDistanceMapPassing.put(velocity, meters);
      firingParametersListPassing.add(
          new ShotCalculatorParameters(
              params.getFlywheelRPS(),
              params.getHoodAngleDeg(),
              params.getTimeOfFlightSec(),
              params.getTunnelRPS(),
              params.getIndexerRPS(),
              meters));
    }

    static {
      // Meters is center of turret to 3 inches behind center from hub

      // Shooting
      putFiringMapEntryScoring(1.560, new FiringParameters(46, 7, 1.3, 45, 9));
      putFiringMapEntryScoring(2.4, new FiringParameters(46, 13, 1.15, 45, 9));
      putFiringMapEntryScoring(3.427, new FiringParameters(52, 17, 1.25, 45, 9));
      putFiringMapEntryScoring(4.6, new FiringParameters(56.5, 22.5, 1.2, 45, 9));
      putFiringMapEntryScoring(5.13, new FiringParameters(60.5, 21, 1.35, 45, 9));
      putFiringMapEntryScoring(5.817, new FiringParameters(63, 28, 1.35, 45, 9));

      // Passing
      putFiringMapEntryPassing(4.60, new FiringParameters(43, 30, 1.5, 45, 7));
      putFiringMapEntryPassing(7.349, new FiringParameters(62, 34, 1.4, 45, 7));
      putFiringMapEntryPassing(10.31, new FiringParameters(80, 38, 1.4, 45, 7));
    }

    // can't maintain burst for full field passes due to battery voltage drop
    public static final boolean alwaysTargetAllianceZone = false;
  }

  public static final double fixedSolutionBlueDeg = 0;
  // Trench structure - distance 2.90
  public static final FiringSolution fixedSolutionBlue =
      new FiringSolution(50.649, 16.099, fixedSolutionBlueDeg, 55, 11);
  public static final FiringSolution fixedSolutionRed =
      new FiringSolution(50.649, 16.099, fixedSolutionBlueDeg + 180, 55, 11);

  public static class FiringTargetTranslations {
    // Right/left are determined as view from blue alliance driver station
    // TODO get exact values
    public static class Red {
      public static final Translation2d hubTranslation = FieldConstants.Red.hubTranslation;
      public static final Translation2d allianceRightTranslation = new Translation2d(15.5, 1);
      public static final Translation2d allianceLeftTranslation = new Translation2d(15.5, 7);
      public static final Translation2d neutralRightTranslation = new Translation2d(8.25, 1.75);
      public static final Translation2d neutralLeftTranslation = new Translation2d(8.25, 6.25);
    }

    public static class Blue {
      public static final Translation2d hubTranslation = FieldConstants.Blue.hubTranslation;
      public static final Translation2d allianceRightTranslation = new Translation2d(1, 1);
      public static final Translation2d allianceLeftTranslation = new Translation2d(1, 7);
      public static final Translation2d neutralRightTranslation = new Translation2d(8.25, 1.75);
      public static final Translation2d neutralLeftTranslation = new Translation2d(8.25, 6.25);
    }
  }

  public static class Systemcore {
    public static final CANBus bus0 = new CANBus(CANPort.CAN_S0);
    public static final CANBus bus1 = new CANBus(CANPort.CAN_S1);
    public static final CANBus bus2 = new CANBus(CANPort.CAN_S2);
    public static final CANBus bus3 = new CANBus(CANPort.CAN_S3);
    public static final CANBus bus4 = new CANBus(CANPort.CAN_S4);
  }

  public static class HubTracker {

    public static final int preBuffer = 3; // TODO
    public static final int postBuffer = 2;
  }

  public static class Sim {

    public static final double tunnelRate = 2;
    public static final double spindexerRate = 0.2;
    public static final double flywheelRate = 2;
    public static final double hoodRate = 2;
  }

  public static final class VisionGlobalPose {
    // See if this helps with NT/CPU stability
    public static final boolean enableVerbosePoseLogging = false;
    // Camera names, must match names configured on coprocessor
    public static String frontRightName = "FrontRight";
    public static String frontLeftName = "FrontLeft";
    public static String backRightName = "BackRight";
    public static String backLeftName = "BackLeft";

    // Robot to camera transforms
    // TODO
    public static Transform3d frontRightTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(14.199859),
                Units.inchesToMeters(-12.199859),
                Units.inchesToMeters(21.455136)),
            new Rotation3d(0, Units.degreesToRadians(-19.8534025106), Units.degreesToRadians(-45)));
    public static Transform3d frontLeftTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(14.199859),
                Units.inchesToMeters(12.199859),
                Units.inchesToMeters(21.455136)),
            new Rotation3d(0, Units.degreesToRadians(-19.8534025106), Units.degreesToRadians(45)));
    public static Transform3d backRightTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-14.199859),
                Units.inchesToMeters(-12.199859),
                Units.inchesToMeters(21.455136)),
            new Rotation3d(
                0, Units.degreesToRadians(-19.8534025106), Units.degreesToRadians(-135)));
    public static Transform3d backLeftTransform =
        new Transform3d(
            new Translation3d(
                Units.inchesToMeters(-14.199859),
                Units.inchesToMeters(12.199859),
                Units.inchesToMeters(21.455136)),
            new Rotation3d(0, Units.degreesToRadians(-19.8534025106), Units.degreesToRadians(135)));

    // Basic filtering thresholds
    public static double maxAmbiguity = 0.15;
    public static double maxZError = 0.75;
    public static double maxAvgTagDistance = 30; // Basically allow tags from any distance
    public static double stdDevBaseline = 0.2;
    public static double singleTagStdDevAdjuster = 3;
  }

  public static final class VisionObjectDetection {
    public enum ObjectDetectionTarget {
      CENTROID,
      CLOSEST
    }

    public static final ObjectDetectionTarget mode = ObjectDetectionTarget.CLOSEST;

    public static final ObjectDetectionType detectionType = ObjectDetectionType.OBJECT;
    public static final Transform3d robotCenterToCamera =
        new Transform3d(
            -0.2208,
            -0.23495,
            0.98315,
            new Rotation3d(0, Units.degreesToRadians(40), Units.degreesToRadians(180)));
    public static final boolean enableObjectDetectionDebug = false;
    public static final String objectCamera = "objectCamera1";

    public static final double fuelIntakeOffset = Units.inchesToMeters(13); // TODO set this
    ; // TODO set this
  }

  public static class Autonomous {

    public static final double shootStopTime = 0.0;
    public static final double pathPlannerDrivekP = 5; // TODO probably increase
    public static final double pathPlannerRotationkP = 4;
    public static final double unjamTimeSec = 0.5;
    public static final double twoSweepSmooshDelayFirstPass = 1.5;
    public static final double smooshDelaySecondPass = 2.5;
    public static final double smooshDelaySinglePass = 1.5;

    public static final double twoSweepShootTimeFirstPass = 3.0;
    public static final double shootTimeSecondPass = 2.0;
    public static final double shootTimeSinglePass = 3.0;
    public static final double outpostWaitTime = 4.0;
    public static final double smooshDelayFirst2056 = 1.0;
    public static final double smooshDelaySecond2056 = 1.0;
    public static final double intakeDelay = 0.5;
    public static final double thirdSweepTimeMin = 6;
  }

  public static class LED {
    public static final int CANdleID = 99;
    public static final StripTypeValue stripType = StripTypeValue.RGBW; // TODO set these
    public static final double brightnessScalar = 0.5;
    public static final int ledStart = 0;
    public static final int ledEnd = 0;
  }

  public static class NetworkTables {
    public static final Color red = new Color(255, 0, 0);
    public static final Color green = new Color(0, 255, 0);
    public static final Color yellow = new Color(255, 255, 0);
  }

  public static class ShotCalculator {
    public static final double minConfidence = 1;

    public static double phaseDelayMs = 30.0; // your vision pipeline latency
    public static double mechLatencyMs = 100.0; // how long the mechanism takes to respond
    public static double maxTiltDeg =
        5.0; // suppress firing when chassis tilts past this (bumps/ramps)
    public static double headingSpeedScalar =
        1.0; // heading tolerance tightens with robot speed (0 to disable)
    public static double headingReferenceDistance =
        2.5; // heading tolerance scales with distance from hub
  }
}
