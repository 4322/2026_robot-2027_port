package frc.robot.subsystems;

import org.wpilib.hardware.hal.AllianceStationID;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Timer;
import org.wpilib.driverstation.XboxController;
import org.wpilib.simulation.DriverStationSim;
import org.wpilib.command2.InstantCommand;
import frc.robot.RobotContainer;
import frc.robot.autonomous.AutonomousSelector.AutoName;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Map;
import org.littletonrobotics.junction.Logger;

public class Simulator {
  private static final RegressTests regressTest = RegressTests.DO_NOTHING;
  public static AutoName autoScenario;
  private TeleopScenario teleopScenario;
  private List<TeleAnomaly> teleAnomalies;
  private List<AutoAnomaly> autoAnomalies;
  private String currentScenario;
  private String gameSpecificMessage = "";
  private final Map<Integer, Double> axisValues = new HashMap<Integer, Double>();
  private Timer warmupTimer = new Timer();
  public static boolean slipWheels = false;

  private enum RegressTests {
    DO_NOTHING,
    SHOOT,
    CONTROLLER_TEST,
    SUBSYSTEM_TEST_BOTH,
    SUBSYSTEM_TEST_TELE,
    TEST_AUTOROTATE,
    AUTO,
    TURRET,
    ALL_AUTOS,
    ZONES,
    INTAKE_TEST,
    DRIVE_WHILE_SHOOTING,
    TRENCHES
  }

  private enum TeleAnomaly {
    NONE
  }

  private enum AutoAnomaly {
    NONE
  }

  private enum TeleopScenario {
    NONE,
    SHOOT,
    CONTROLLER_TEST1,
    CONTROLLER_TEST2,
    SUBSYSTEM_TEST,
    AUTO_ROTATE,
    DRIVE_WHILE_SHOOTING,
    TURRET,
    Slowly_Up_down,
    ZONES,
    INTAKE_TEST,
    TRENCHES
  }

  private enum EventType {
    SET_POSE,
    PRESS_A,
    HOLD_A,
    RELEASE_A,
    PRESS_B,
    HOLD_B,
    RELEASE_B,
    PRESS_X,
    HOLD_X,
    RELEASE_X,
    PRESS_Y,
    HOLD_Y,
    RELEASE_Y,
    PRESS_LEFT_BUMPER,
    HOLD_LEFT_BUMPER,
    RELEASE_LEFT_BUMPER,
    PRESS_RIGHT_BUMPER,
    HOLD_RIGHT_BUMPER,
    RELEASE_RIGHT_BUMPER,
    RELEASE_POV,
    PRESS_UP_POV,
    HOLD_UP_POV,
    PRESS_RIGHT_POV,
    HOLD_RIGHT_POV,
    PRESS_DOWN_POV,
    HOLD_DOWN_POV,
    PRESS_LEFT_POV,
    HOLD_LEFT_POV,
    HOLD_LEFT_TRIGGER,
    RELEASE_LEFT_TRIGGER,
    HOLD_RIGHT_TRIGGER,
    RELEASE_RIGHT_TRIGGER,
    PRESS_LEFT_STICK,
    HOLD_LEFT_STICK,
    RELEASE_LEFT_STICK,
    PRESS_RIGHT_STICK,
    HOLD_RIGHT_STICK,
    RELEASE_RIGHT_STICK,
    MOVE_JOYSTICK_DRIVE,
    MOVE_JOYSTICK_TURN,
    STOP_JOYSTICK,
    ENABLE_WHEEL_SLIP,
    DISABLE_WHEEL_SLIP,
    BLUE_INACTIVE_FIRST,
    RED_INACTIVE_FIRST,
    END_OF_SCENARIO
  }

  private enum EventStatus {
    ACTIVE,
    INACTIVE
  }

  private enum POVDirection {
    NONE(-1),
    UP(0),
    RIGHT(90),
    DOWN(180),
    LEFT(270);

    public final int value;

    private POVDirection(int value) {
      this.value = value;
    }
  }

  private enum ControllerAxis {
    LEFT_X(0),
    LEFT_Y(1),
    LEFT_TRIGGER(2),
    RIGHT_TRIGGER(3),
    RIGHT_X(4),
    RIGHT_Y(5);

    public final int value;

    private ControllerAxis(int value) {
      this.value = value;
    }
  }

  public class FieldPose2d extends Pose2d {
    public FieldPose2d(double x, double y, Rotation2d rotation) {
      super(x, y, rotation);
    }

    public Pose2d flipPose() {
      double flippedX = Units.inchesToMeters(651.22) - getX();
      double flippedY = Units.inchesToMeters(317.69) - getY();
      return new Pose2d(flippedX, flippedY, getRotation().plus(Rotation2d.kPi));
    }
  }

  private static class RegressionTest {
    private String name;
    private AutoName autoScenario;
    private List<AutoAnomaly> autoAnomalies;
    private TeleopScenario teleopScenario;
    private List<TeleAnomaly> teleAnomalies;
    private Alliance alliance;

    RegressionTest(
        String name,
        AutoName autoScenario,
        List<AutoAnomaly> autoAnomalies,
        TeleopScenario teleopScenario,
        List<TeleAnomaly> teleAnomalies,
        Alliance alliance) {
      this.name = name;
      this.autoScenario = autoScenario;
      this.autoAnomalies = autoAnomalies;
      this.teleopScenario = teleopScenario;
      this.teleAnomalies = teleAnomalies;
      this.alliance = alliance;
    }

    @SuppressWarnings("unused")
    RegressionTest(
        String name, AutoName autoScenario, List<AutoAnomaly> autoAnomalies, Alliance alliance) {
      this(name, autoScenario, autoAnomalies, null, null, alliance);
    }

    @SuppressWarnings("unused")
    RegressionTest(
        String name,
        AutoName autoScenario,
        TeleopScenario teleopScenario,
        List<TeleAnomaly> teleAnomalies,
        Alliance alliance) {
      this(name, autoScenario, (List<AutoAnomaly>) null, teleopScenario, teleAnomalies, alliance);
    }

    @SuppressWarnings("unused")
    RegressionTest(
        String name, AutoName autoScenario, TeleopScenario teleopScenario, Alliance alliance) {
      this(name, autoScenario, teleopScenario, null, alliance);
    }

    @SuppressWarnings("unused")
    RegressionTest(String name, AutoName autoScenario, Alliance alliance) {
      this(name, autoScenario, null, null, alliance);
    }

    RegressionTest(
        String name,
        TeleopScenario teleopScenario,
        List<TeleAnomaly> teleAnomalies,
        Alliance alliance) {
      this(name, null, teleopScenario, teleAnomalies, alliance);
    }

    RegressionTest(String name, TeleopScenario teleopScenario, Alliance alliance) {
      this(name, teleopScenario, null, alliance);
    }
  }

  private List<RegressionTest> regressionTestCases() {
    return switch (regressTest) {
      case AUTO -> List.of(new RegressionTest("AUTO", AutoName.R_2_SWEEP, Alliance.Blue));
      case SHOOT -> List.of(new RegressionTest("Shoot", TeleopScenario.SHOOT, Alliance.Blue));
      case DO_NOTHING -> List.of(
          new RegressionTest("Do nothing", AutoName.DO_NOTHING, Alliance.Blue));
      case CONTROLLER_TEST -> List.of(
          new RegressionTest("Controller Test 1", TeleopScenario.CONTROLLER_TEST1, Alliance.Blue),
          new RegressionTest("Controller Test 2", TeleopScenario.CONTROLLER_TEST2, Alliance.Blue));
      case SUBSYSTEM_TEST_BOTH -> List.of(
          new RegressionTest(
              "Auto test", AutoName.R_2_SWEEP, TeleopScenario.AUTO_ROTATE, Alliance.Blue));
      case SUBSYSTEM_TEST_TELE -> List.of(
          new RegressionTest("Subsystem Test", TeleopScenario.SUBSYSTEM_TEST, Alliance.Blue));
      case TEST_AUTOROTATE -> List.of(
          new RegressionTest("Auto Rotate", TeleopScenario.AUTO_ROTATE, Alliance.Blue));
      case TURRET -> List.of(
          new RegressionTest("Turret Test", TeleopScenario.TURRET, Alliance.Blue));
      case ALL_AUTOS -> List.of();
      case ZONES -> List.of(
          new RegressionTest("Zones Blue", TeleopScenario.ZONES, Alliance.Blue),
          new RegressionTest("Zones Red", TeleopScenario.ZONES, Alliance.Red));
      case INTAKE_TEST -> List.of(
          new RegressionTest("Intake Test", TeleopScenario.INTAKE_TEST, Alliance.Blue));
      case DRIVE_WHILE_SHOOTING -> List.of(
          new RegressionTest(
              "Drive While Shooting", TeleopScenario.DRIVE_WHILE_SHOOTING, Alliance.Blue));
      case TRENCHES -> List.of(
          new RegressionTest("Trenches", TeleopScenario.TRENCHES, Alliance.Blue),
          new RegressionTest("Trenches", TeleopScenario.TRENCHES, Alliance.Red));
      default -> List.of();
    };
  }

  private class SimEvent {
    private double eventTime;
    private EventStatus eventStatus;
    private String eventName;
    private EventType eventType;
    private Pose2d pose;

    SimEvent(double eventTime, String eventName, EventType eventType) {
      this(eventTime, eventName, eventType, EventStatus.ACTIVE);
    }

    SimEvent(double eventTime, String eventName, EventType eventType, EventStatus eventStatus) {
      this(eventTime, eventName, eventType, (Translation2d) null, eventStatus);
    }

    @SuppressWarnings("unused")
    SimEvent(double eventTime, String eventName, EventType eventType, Translation2d position) {
      this(eventTime, eventName, eventType, new Pose2d(position, Rotation2d.kZero));
    }

    SimEvent(
        double eventTime,
        String eventName,
        EventType eventType,
        Translation2d position,
        EventStatus eventStatus) {
      this(eventTime, eventName, eventType, new Pose2d(position, Rotation2d.kZero), eventStatus);
    }

    SimEvent(double eventTime, String eventName, EventType eventType, Pose2d pose) {
      this(eventTime, eventName, eventType, pose, EventStatus.ACTIVE);
    }

    SimEvent(
        double eventTime,
        String eventName,
        EventType eventType,
        Pose2d pose,
        EventStatus eventStatus) {
      this.eventTime = eventTime;
      this.eventName = eventName;
      this.eventType = eventType;
      this.eventStatus = eventStatus;
      this.pose =
          currentAlliance == Alliance.Blue || !(pose instanceof FieldPose2d)
              ? pose
              : ((FieldPose2d) pose).flipPose();
    }
  }

  private List<SimEvent> buildAutoScenario() {
    if (autoScenario == null) {
      return List.of();
    }
    double t = 0.0;
    return switch (autoScenario) {
      default -> List.of(new SimEvent(t += 20.0, "Final Movement", EventType.END_OF_SCENARIO));
    };
  }

  private List<SimEvent> buildTeleopScenario() {
    if (teleopScenario == null) {
      return List.of();
    }
    double t = 0.0;
    int eventNum = 1;

    return switch (teleopScenario) {
      case CONTROLLER_TEST1 -> List.of(
          new SimEvent(
              t += 1.0,
              "Start pose",
              EventType.SET_POSE,
              new FieldPose2d(12.0, 4.0, Rotation2d.k180deg)),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_X),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_LEFT_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_RIGHT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_X),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_X),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.3, 0.3, Rotation2d.k180deg)),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(0, 0.3, Rotation2d.k180deg)),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.RELEASE_RIGHT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.STOP_JOYSTICK),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_UP_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_BUMPER),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.3, 0.3, Rotation2d.kZero)),
          // Turn X isn't used, setting it here to test controller data reset
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(1, -0.3, Rotation2d.kZero)),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_X),
          new SimEvent(t += 1.0, "Final Movement", EventType.END_OF_SCENARIO));

      case CONTROLLER_TEST2 -> List.of(
          // check that controls were released from CONTROLLER_TEST1 when switching to this scenario
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_X),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_RIGHT_POV),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_LEFT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_LEFT_BUMPER),
          new SimEvent(t += 1.0, "Event " + eventNum++, EventType.PRESS_RIGHT_BUMPER),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.4, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 1.0,
              "Event " + eventNum++,
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(0, -0.3, Rotation2d.kZero)),
          new SimEvent(t += 1.0, "Final Movement", EventType.END_OF_SCENARIO));

      case SUBSYSTEM_TEST -> List.of(
          new SimEvent(
              t, "Start Pose", EventType.SET_POSE, new FieldPose2d(4.44, 0.650, Rotation2d.kZero)),
          new SimEvent(t += 4.0, "Start Intake", EventType.PRESS_Y),
          new SimEvent(t += 1.0, "Stop Intake", EventType.PRESS_X),
          new SimEvent(
              t += 2.0,
              "Drive 1",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.5, 0, Rotation2d.k180deg)),
          new SimEvent(
              t += 1.0,
              "Drive 2",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.5, 0.5, Rotation2d.k180deg)),
          new SimEvent(
              t, "Turn 1", EventType.MOVE_JOYSTICK_TURN, new Pose2d(0, -0.5, Rotation2d.kZero)),
          new SimEvent(
              t += 1.5,
              "Drive 3",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0, -0.5, Rotation2d.k180deg)),
          new SimEvent(
              t, "Turn 2", EventType.MOVE_JOYSTICK_TURN, new Pose2d(0, 0.5, Rotation2d.kZero)),
          new SimEvent(
              t += 1.5, "Turn 3", EventType.MOVE_JOYSTICK_TURN, new Pose2d(0, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 0.75,
              "Drive 4",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.8, 0, Rotation2d.k180deg)),
          new SimEvent(
              t += 2,
              "Drive 5",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.5, 0.5, Rotation2d.k180deg)),
          new SimEvent(
              t += 0.75,
              "Drive 6",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.5, 0.5, Rotation2d.k180deg)),
          new SimEvent(
              t += 1,
              "Drive 7",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0, 0.75, Rotation2d.k180deg)),
          new SimEvent(
              t += 1,
              "Drive 8",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0, 0, Rotation2d.k180deg)),
          new SimEvent(t += 3.0, "Final Movement", EventType.END_OF_SCENARIO));

      case AUTO_ROTATE -> List.of(
          new SimEvent(
              t += 1.0, "Start pose", EventType.SET_POSE, new FieldPose2d(2, 2, Rotation2d.kZero)),
          new SimEvent(
              t += 1,
              "Drive to Neutral Zone",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.5, 0.5, Rotation2d.k180deg)),
          new SimEvent(
              t += 1.5,
              "Stop Drive",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.0, 0.0, Rotation2d.k180deg)),
          new SimEvent(t += 0.1, "Shoot", EventType.HOLD_RIGHT_BUMPER),
          new SimEvent(t += 6, "Stop Shooting", EventType.RELEASE_RIGHT_BUMPER),
          new SimEvent(
              t += 0.1,
              "Drive Back",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.5, -0.5, Rotation2d.k180deg)),
          new SimEvent(
              t += 1.5,
              "Drive to Opposing Zone",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.67, 0.0, Rotation2d.k180deg)),
          new SimEvent(
              t += 4,
              "Stop Drive",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.0, 0.0, Rotation2d.k180deg)),
          new SimEvent(t += 0.1, "Shoot", EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(t += 6, "Stop Shooting", EventType.RELEASE_RIGHT_TRIGGER),
          new SimEvent(
              t += 0.1,
              "Drive Back",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-0.67, 0, Rotation2d.k180deg)),
          new SimEvent(
              t += 4,
              "Stop Drive",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.0, 0.0, Rotation2d.k180deg)),
          new SimEvent(t += 0.1, "Shoot", EventType.HOLD_RIGHT_BUMPER),
          new SimEvent(t += 6, "Stop Shooting", EventType.RELEASE_RIGHT_BUMPER),
          new SimEvent(
              t += 0,
              "Shoot on the move",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0, 0.3, Rotation2d.k180deg)),
          new SimEvent(t += 0.1, "Shoot", EventType.HOLD_RIGHT_BUMPER),
          new SimEvent(t += 6, "Stop Shooting", EventType.RELEASE_RIGHT_BUMPER),
          new SimEvent(t += 0.1, "Final Movement", EventType.END_OF_SCENARIO));

      case TURRET -> List.of(
          // requires turret to be unlocked
          new SimEvent(t += 0.1, "Start intake", EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 0.1, "Release button", EventType.RELEASE_LEFT_BUMPER),
          new SimEvent(t += 0.5, "Start shooting", EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(
              t += 0.1, "Start pose", EventType.SET_POSE, new FieldPose2d(2, 2, Rotation2d.kZero)),
          new SimEvent(
              t += 0.1,
              "Spin",
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(0.0, 0.3, Rotation2d.kZero)),
          new SimEvent(t += 1.0, "Start unjam button", EventType.HOLD_RIGHT_BUMPER),
          new SimEvent(t += 5.0, "Start unjam button", EventType.RELEASE_RIGHT_BUMPER),
          new SimEvent(
              t += 20.0,
              "Spin",
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(0.0, -0.3, Rotation2d.kZero)),
          new SimEvent(t += 40.0, "Final Movement", EventType.END_OF_SCENARIO));
      case Slowly_Up_down -> List.of(
          new SimEvent(
              t += 0,
              "Shoot on the move",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0, 0.3, Rotation2d.k180deg)),
          new SimEvent(
              t += 0,
              "Shoot on the move",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0, -0.3, Rotation2d.k180deg)),
          new SimEvent(
              t += 0,
              "Shoot on the move",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0, 0.3, Rotation2d.k180deg)),
          new SimEvent(
              t += 0,
              "Shoot on the move",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0, -0.3, Rotation2d.k180deg)));

      case ZONES -> List.of(
          new SimEvent(
              t += 0, "Lateral", EventType.SET_POSE, new FieldPose2d(3.7, -0.5, Rotation2d.kZero)),
          new SimEvent(
              t += 0.1,
              "Move",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0, 0.42, Rotation2d.kZero)),
          new SimEvent(
              t += 10, "Stop", EventType.MOVE_JOYSTICK_DRIVE, new Pose2d(0, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 0.1,
              "Right Trench",
              EventType.SET_POSE,
              new FieldPose2d(-0.5, 0.5, Rotation2d.kZero)),
          new SimEvent(
              t += 0.1,
              "Move",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.46, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 10, "Stop", EventType.MOVE_JOYSTICK_DRIVE, new Pose2d(0, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 0.1,
              "Left Trench",
              EventType.SET_POSE,
              new FieldPose2d(-0.5, 7.5, Rotation2d.kZero)),
          new SimEvent(
              t += 0.1,
              "Move",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0.46, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 10, "Stop", EventType.MOVE_JOYSTICK_DRIVE, new Pose2d(0, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 0.1,
              "Fixed Pose",
              EventType.SET_POSE,
              new FieldPose2d(3.7, 6.5, Rotation2d.kZero)),
          new SimEvent(t += 0.1, "Fixed Shoot", EventType.HOLD_RIGHT_BUMPER),
          new SimEvent(t += 5, "End", EventType.END_OF_SCENARIO));
      case INTAKE_TEST -> List.of(
          new SimEvent(
              t += 0.1, "Start pose", EventType.SET_POSE, new FieldPose2d(2, 2, Rotation2d.kZero)),
          new SimEvent(t += 0.5, "Deploy/Intake", EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 0.5, "Deploy/Intake Release", EventType.RELEASE_LEFT_BUMPER),
          new SimEvent(t += 0.5, "IntakeOff", EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 0.5, "IntakeOff Release", EventType.RELEASE_LEFT_BUMPER),
          new SimEvent(t += 0.5, "Start ejecting", EventType.HOLD_X),
          new SimEvent(t += 2.0, "Stop ejecting", EventType.RELEASE_X),
          new SimEvent(t += 0.5, "Start smooshing", EventType.HOLD_Y),
          new SimEvent(t += 2.0, "Stop smooshing", EventType.RELEASE_Y),
          new SimEvent(t += 0.5, "Start intaking", EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 0.5, "Start ejecting while intaking", EventType.HOLD_X),
          new SimEvent(t += 2.0, "Stop ejecting", EventType.RELEASE_X),
          new SimEvent(t += 0.5, "Start smooshing while intaking", EventType.HOLD_Y),
          new SimEvent(t += 2.0, "Stop smooshing", EventType.RELEASE_Y),
          new SimEvent(t += 0.5, "Stop intaking", EventType.RELEASE_LEFT_BUMPER),
          new SimEvent(t += 0.1, "End", EventType.END_OF_SCENARIO));
      case DRIVE_WHILE_SHOOTING -> List.of(
          new SimEvent(
              t += 0.1,
              "Start pose",
              EventType.SET_POSE,
              new FieldPose2d(3, 0.5, Rotation2d.kZero)),
          new SimEvent(t += 0.1, "Start intake button", EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 0.1, "Release intake button", EventType.RELEASE_LEFT_BUMPER),
          new SimEvent(
              t += 0.1,
              "Move Left No Shoot",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0, 1, Rotation2d.kZero)),
          new SimEvent(
              t += 3, "Stop", EventType.MOVE_JOYSTICK_DRIVE, new Pose2d(0, 0, Rotation2d.kZero)),
          new SimEvent(t += 0.1, "Start shooting", EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(
              t += 2,
              "Move Right Shoot",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(0, -1, Rotation2d.kZero)),
          new SimEvent(
              t += 3.5, "Stop", EventType.MOVE_JOYSTICK_DRIVE, new Pose2d(0, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 0.1,
              "Spin Shoot",
              EventType.MOVE_JOYSTICK_TURN,
              new Pose2d(0, 1, Rotation2d.kZero)),
          new SimEvent(
              t += 10, "Stop", EventType.MOVE_JOYSTICK_TURN, new Pose2d(0, 0, Rotation2d.kZero)),
          new SimEvent(t += 0.1, "Stop shooting", EventType.RELEASE_RIGHT_TRIGGER),
          new SimEvent(t += 0.1, "End", EventType.END_OF_SCENARIO));
      case TRENCHES -> List.of(
          new SimEvent(
              t += 0.1, "Start pose", EventType.SET_POSE, new FieldPose2d(3, 1, Rotation2d.kZero)),
          new SimEvent(t += 0.1, "Deploy intake", EventType.HOLD_LEFT_BUMPER),
          new SimEvent(t += 3, "Shoot", EventType.HOLD_RIGHT_TRIGGER),
          new SimEvent(
              t += 0.1,
              "Go through right blue trench",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(1, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 5,
              "Go back through right blue trench",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-1, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 5,
              "Left blue trench pose",
              EventType.SET_POSE,
              new FieldPose2d(3, 8, Rotation2d.kZero)),
          new SimEvent(
              t += 0.1,
              "Go through left blue trench",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(1, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 5,
              "Go back through left blue trench",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-1, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 5,
              "Right red trench pose",
              EventType.SET_POSE,
              new FieldPose2d(13, 1, Rotation2d.kZero)),
          new SimEvent(
              t += 0.1,
              "Go through right red trench",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-1, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 5,
              "Go back through right red trench",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(1, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 5,
              "Left red trench pose",
              EventType.SET_POSE,
              new FieldPose2d(13, 8, Rotation2d.kZero)),
          new SimEvent(
              t += 0.1,
              "Go through left red trench",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(-1, 0, Rotation2d.kZero)),
          new SimEvent(
              t += 5,
              "Go back through left red trench",
              EventType.MOVE_JOYSTICK_DRIVE,
              new Pose2d(1, 0, Rotation2d.kZero)),
          new SimEvent(t += 5, "End", EventType.END_OF_SCENARIO));
      default -> List.of();
    };
  }

  @SuppressWarnings("unused")
  private boolean aListContains(AutoAnomaly aAnomaly) {
    if (autoAnomalies == null) {
      return false;
    }
    return autoAnomalies.contains(aAnomaly);
  }

  @SuppressWarnings("unused")
  private boolean tListContains(TeleAnomaly tAnomaly) {
    if (teleAnomalies == null) {
      return false;
    }
    return teleAnomalies.contains(tAnomaly);
  }

  private Iterator<RegressionTest> regressionTestIterator = regressionTestCases().iterator();
  private RegressionTest currentRegressionTest;
  private List<SimEvent> autoEvents;
  private List<SimEvent> teleopEvents;
  private List<SimEvent> events;
  private Iterator<SimEvent> eventIterator;
  private SimEvent currentEvent;
  private Alliance currentAlliance;
  private final Timer disabledTimer = new Timer();
  private final Timer matchTimer = new Timer();
  XboxController hid = RobotContainer.controller.getHID(); // the real WPILib XboxController
  int hidPort = hid.getPort();
  int activeButtonBitmask;
  int momentaryButtonBitmask;
  int activePOV;
  boolean momentaryPOV;

  public Simulator() {
    warmupTimer.start();
  }

  public void periodic() {
    // wait for PathPanner and other libraries to initialize
    if (warmupTimer.isRunning()) {
      if (warmupTimer.hasElapsed(4)) {
        warmupTimer.stop();
        configureControllerTestBindings();
        setNextRegressTest();
      } else {
        // keep feeding the simulated controller
        configureController();
        DriverStationSim.notifyNewData();
        return;
      }
    }
    if (disabledTimer.isRunning()) {
      if (!disabledTimer.hasElapsed(2)) {
        // keep feeding the simulated controller
        configureController();
        DriverStationSim.notifyNewData();
        return;
      }
      disabledTimer.stop();
      disabledTimer.reset();
      DriverStationSim.setEnabled(true);
      matchTimer.restart();
      if (eventIterator.hasNext()) {
        currentEvent = eventIterator.next();
      } else {
        currentEvent = null;
        if (events == autoEvents && !teleopEvents.isEmpty()) {
          events = teleopEvents;
          resetScenario();
        } else {
          setNextRegressTest();
        }
        return;
      }
    }

    Logger.recordOutput("Sim/MatchTime", matchTimer.get());
    Logger.recordOutput("Sim/RegressionTest", currentRegressionTest.name);
    Logger.recordOutput("Sim/Alliance", currentAlliance.toString());
    Logger.recordOutput("Sim/Scenario", currentScenario);
    Logger.recordOutput("Sim/MatchTime", matchTimer.get());
    configureController();

    // execute all pending events
    while (currentEvent != null && matchTimer.get() >= currentEvent.eventTime) {
      if (currentEvent.eventStatus == EventStatus.ACTIVE) {
        Logger.recordOutput("Sim/EventName", currentEvent.eventName);
        Logger.recordOutput("Sim/EventType", currentEvent.eventType);
        switch (currentEvent.eventType) {
          case SET_POSE -> setPose(currentEvent.pose);
          case PRESS_A -> pressButton(XboxController.Button.kA);
          case HOLD_A -> holdButton(XboxController.Button.kA);
          case RELEASE_A -> releaseButton(XboxController.Button.kA);
          case PRESS_B -> pressButton(XboxController.Button.kB);
          case HOLD_B -> holdButton(XboxController.Button.kB);
          case RELEASE_B -> releaseButton(XboxController.Button.kB);
          case PRESS_X -> pressButton(XboxController.Button.kX);
          case HOLD_X -> holdButton(XboxController.Button.kX);
          case RELEASE_X -> releaseButton(XboxController.Button.kX);
          case PRESS_Y -> pressButton(XboxController.Button.kY);
          case HOLD_Y -> holdButton(XboxController.Button.kY);
          case RELEASE_Y -> releaseButton(XboxController.Button.kY);
          case PRESS_LEFT_BUMPER -> pressButton(XboxController.Button.kLeftBumper);
          case HOLD_LEFT_BUMPER -> holdButton(XboxController.Button.kLeftBumper);
          case RELEASE_LEFT_BUMPER -> releaseButton(XboxController.Button.kLeftBumper);
          case PRESS_RIGHT_BUMPER -> pressButton(XboxController.Button.kRightBumper);
          case HOLD_RIGHT_BUMPER -> holdButton(XboxController.Button.kRightBumper);
          case RELEASE_RIGHT_BUMPER -> releaseButton(XboxController.Button.kRightBumper);
          case RELEASE_POV -> holdPOV(POVDirection.NONE);
          case PRESS_UP_POV -> pressPOV(POVDirection.UP);
          case HOLD_UP_POV -> holdPOV(POVDirection.UP);
          case PRESS_RIGHT_POV -> pressPOV(POVDirection.RIGHT);
          case HOLD_RIGHT_POV -> holdPOV(POVDirection.RIGHT);
          case PRESS_DOWN_POV -> pressPOV(POVDirection.DOWN);
          case HOLD_DOWN_POV -> holdPOV(POVDirection.DOWN);
          case PRESS_LEFT_POV -> pressPOV(POVDirection.LEFT);
          case HOLD_LEFT_POV -> holdPOV(POVDirection.LEFT);
          case HOLD_LEFT_TRIGGER -> holdTrigger(ControllerAxis.LEFT_TRIGGER);
          case RELEASE_LEFT_TRIGGER -> releaseTrigger(ControllerAxis.LEFT_TRIGGER);
          case HOLD_RIGHT_TRIGGER -> holdTrigger(ControllerAxis.RIGHT_TRIGGER);
          case RELEASE_RIGHT_TRIGGER -> releaseTrigger(ControllerAxis.RIGHT_TRIGGER);
          case PRESS_LEFT_STICK -> pressButton(XboxController.Button.kLeftStick);
          case HOLD_LEFT_STICK -> holdButton(XboxController.Button.kLeftStick);
          case RELEASE_LEFT_STICK -> releaseButton(XboxController.Button.kLeftStick);
          case PRESS_RIGHT_STICK -> pressButton(XboxController.Button.kRightStick);
          case HOLD_RIGHT_STICK -> holdButton(XboxController.Button.kRightStick);
          case RELEASE_RIGHT_STICK -> releaseButton(XboxController.Button.kRightStick);
          case MOVE_JOYSTICK_DRIVE -> {
            persistAxis(ControllerAxis.LEFT_X, -currentEvent.pose.getY());
            persistAxis(ControllerAxis.LEFT_Y, -currentEvent.pose.getX());
          }
          case MOVE_JOYSTICK_TURN -> {
            persistAxis(ControllerAxis.RIGHT_X, -currentEvent.pose.getY());
            persistAxis(ControllerAxis.RIGHT_Y, -currentEvent.pose.getX());
          }
          case STOP_JOYSTICK -> stopJoystick();
          case ENABLE_WHEEL_SLIP -> slipWheels = true;
          case DISABLE_WHEEL_SLIP -> slipWheels = false;
          case BLUE_INACTIVE_FIRST -> gameSpecificMessage = "B";
          case RED_INACTIVE_FIRST -> gameSpecificMessage = "R";
          case END_OF_SCENARIO -> {}
        }
      }
      if (eventIterator.hasNext()) {
        currentEvent = eventIterator.next();
      } else {
        // end of scenario
        currentEvent = null;
        if (events == autoEvents && !teleopEvents.isEmpty()) {
          events = teleopEvents;
          resetScenario();
        } else {
          setNextRegressTest();
        }
      }
    }

    // update Xbox controls
    int retry = 0;
    do {
      if (++retry > 5) {
        System.err.println(
            "*** Aborting sim: Xbox controller update retry limit exceeded at matchTime "
                + matchTimer.get());
        System.exit(1);
      }
      configureController();
      for (Map.Entry<Integer, Double> entry : axisValues.entrySet()) {
        DriverStationSim.setJoystickAxis(hidPort, entry.getKey(), entry.getValue());
      }
      DriverStationSim.setGameSpecificMessage(gameSpecificMessage);
      DriverStationSim.setJoystickPOV(hidPort, 0, activePOV);
      DriverStationSim.setJoystickButtons(hidPort, activeButtonBitmask);
      // notifyNewData() fails 1 out of 5000 calls for unknown reasons
      DriverStationSim.notifyNewData();
    } while (DriverStation.getStickAxisCount(hidPort) != 6
        || DriverStation.getStickButtonCount(hidPort) != 10);
    if (retry > 2) {
      System.out.println(
          "Warning: Xbox ontroller update required "
              + retry
              + " attempts at matchTime "
              + matchTimer.get());
    }
    releaseMomentaryButtons();
    if (momentaryPOV) {
      activePOV = POVDirection.NONE.value;
      momentaryPOV = false;
    }
  }

  private void configureController() {
    // ensure controller has expected number of axes and buttons
    DriverStationSim.setJoystickIsXbox(hidPort, true);
    DriverStationSim.setJoystickAxisCount(hidPort, 6);
    DriverStationSim.setJoystickButtonCount(hidPort, 10);
    DriverStationSim.setJoystickPOVCount(hidPort, 1);
  }

  private void resetScenario() {
    DriverStationSim.setEnabled(false);
    disabledTimer.start();
    Logger.recordOutput("Sim/RegressionTest", "Disabled");
    Logger.recordOutput("Sim/Scenario", "Disabled");
    Logger.recordOutput("Sim/EventName", "Disabled");
    Logger.recordOutput("Sim/EventType", "Disabled");

    DriverStationSim.setAutonomous(events == autoEvents);
    if (currentAlliance == Alliance.Red) {
      DriverStationSim.setAllianceStationId(AllianceStationID.Red2);
    } else {
      DriverStationSim.setAllianceStationId(AllianceStationID.Blue2);
    }

    eventIterator = events.iterator();
    if (events == autoEvents) {
      currentScenario = autoScenario.toString();
    } else {
      currentScenario = teleopScenario.toString();
    }
  }

  private void setNextRegressTest() {
    // reset all controls
    // don't call DriverStationSim.resetData() because it creates race conditions
    holdPOV(POVDirection.NONE);
    releaseTrigger(ControllerAxis.LEFT_TRIGGER);
    releaseTrigger(ControllerAxis.RIGHT_TRIGGER);
    stopJoystick();
    activeButtonBitmask = 0;
    momentaryButtonBitmask = 0;
    gameSpecificMessage = "";
    DriverStationSim.setEnabled(false);

    if (!regressionTestIterator.hasNext()) {
      System.out.println("All tests complete");
      System.exit(0);
    }
    currentRegressionTest = regressionTestIterator.next();
    autoScenario = currentRegressionTest.autoScenario;
    autoAnomalies = currentRegressionTest.autoAnomalies;
    teleopScenario = currentRegressionTest.teleopScenario;
    teleAnomalies = currentRegressionTest.teleAnomalies;
    currentAlliance = currentRegressionTest.alliance;
    autoEvents = buildAutoScenario();
    teleopEvents = buildTeleopScenario();
    if (!autoEvents.isEmpty()) {
      events = autoEvents;
    } else {
      events = teleopEvents;
    }
    setPose(new Pose2d(0, 0, Rotation2d.kZero));
    resetScenario();
  }

  private void setPose(Pose2d robotPose) {
    RobotContainer.drive.setPose(robotPose);
    RobotContainer.visionGlobalPose.setRobotPose(robotPose);
  }

  private void holdButton(XboxController.Button button) {
    activeButtonBitmask |= 1 << (button.value - 1);
  }

  private void releaseButton(XboxController.Button button) {
    activeButtonBitmask &= ~(1 << (button.value - 1));
  }

  private void pressButton(XboxController.Button button) {
    activeButtonBitmask |= 1 << (button.value - 1);
    momentaryButtonBitmask |= 1 << (button.value - 1);
  }

  private void releaseMomentaryButtons() {
    activeButtonBitmask &= ~momentaryButtonBitmask;
    momentaryButtonBitmask = 0;
  }

  private void holdPOV(POVDirection direction) {
    activePOV = direction.value;
  }

  private void pressPOV(POVDirection direction) {
    holdPOV(direction);
    momentaryPOV = true;
  }

  public void persistAxis(ControllerAxis axis, double value) {
    axisValues.put(axis.value, value);
  }

  private void stopJoystick() {
    persistAxis(ControllerAxis.LEFT_X, 0.0);
    persistAxis(ControllerAxis.LEFT_Y, 0.0);
    persistAxis(ControllerAxis.RIGHT_X, 0.0);
    persistAxis(ControllerAxis.RIGHT_Y, 0.0);
  }

  private void holdTrigger(ControllerAxis axis) {
    persistAxis(axis, 1.0);
  }

  private void releaseTrigger(ControllerAxis axis) {
    persistAxis(axis, 0.0);
  }

  public static boolean wheelSlip() {
    return slipWheels;
  }

  public static AutoName getAutoScenario() {
    return autoScenario;
  }

  private void configureControllerTestBindings() {
    RobotContainer.controller
        .x()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Button X True");
                }));
    RobotContainer.controller
        .x()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Button X False");
                }));
    RobotContainer.controller
        .povLeft()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left POV True");
                }));
    RobotContainer.controller
        .povLeft()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left POV False");
                }));
    RobotContainer.controller
        .leftBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left Bumper True");
                }));
    RobotContainer.controller
        .leftBumper()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left Bumper False");
                }));
    RobotContainer.controller
        .rightBumper()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Right Bumper True");
                }));
    RobotContainer.controller
        .rightBumper()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Right Bumper False");
                }));
    RobotContainer.controller
        .leftTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left Trigger True");
                }));
    RobotContainer.controller
        .leftTrigger()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Left Trigger False");
                }));
    RobotContainer.controller
        .rightTrigger()
        .onTrue(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Right Trigger True");
                }));
    RobotContainer.controller
        .rightTrigger()
        .onFalse(
            new InstantCommand(
                () -> {
                  Logger.recordOutput("Sim/Debug", "Right Trigger False");
                }));
  }
}
