package frc.robot.autonomous;

import org.wpilib.command2.Command;
import org.wpilib.command2.Commands;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.sysid.SysIdRoutine;
import org.wpilib.tunable.Selectable;

import frc.robot.autonomous.modes.CenterPreload;
import frc.robot.autonomous.modes.CenterStartToDepot;
import frc.robot.autonomous.modes.DoNothing;
import frc.robot.autonomous.modes.L2Sweep;
import frc.robot.autonomous.modes.L2SweepDepot;
import frc.robot.autonomous.modes.LSingleSweep;
import frc.robot.autonomous.modes.R2Sweep;
import frc.robot.autonomous.modes.RSingleSweep;
import frc.robot.autonomous.modes.secondShallow.LSecondShallowBump;
import frc.robot.autonomous.modes.secondShallow.LSecondShallowTrench;
import frc.robot.autonomous.modes.secondShallow.RSecondShallowBump;
import frc.robot.autonomous.modes.secondShallow.RSecondShallowBumpDepot;
import frc.robot.autonomous.modes.secondShallow.RSecondShallowTrench;
import frc.robot.autonomous.modes.secondShallow.RSecondShallowTrenchDepot;
import frc.robot.commands.DriveCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Mode;
import frc.robot.subsystems.Simulator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;
import frc.robot.util.LoggedTunableNumber;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class AutonomousSelector {

  private Selectable<Command> autonomousSelector =
      new Selectable<Command>();

  private static LoggedTunableNumber autoStartDelay =
      new LoggedTunableNumber("Auto Start Delay", 0, true);

  public enum AutoName {
    DO_NOTHING,
    R_2_SWEEP,
    L_2_SWEEP,
    L_2_SWEEP_DEPOT,
    L_SINGLE_SWEEP,
    R_SINGLE_SWEEP,

    C_START_TO_DEPOT,
    C_PRELOAD,

    R_SECOND_SHALLOW_BUMP,
    R_SECOND_SHALLOW_TRENCH,
    R_SECOND_SHALLOW_BUMP_DEPOT,
    R_SECOND_SHALLOW_TRENCH_DEPOT,
    L_SECOND_SHALLOW_BUMP,
    L_SECOND_SHALLOW_TRENCH,

    DRIVE_WHEEL_RADIUS_CHARACTERIZATION,
    DRIVE_SIMPLE_FF_CHARACTERIZATION,
    DRIVE_SYS_ID_QUASISTATIC_FORWARD,
    DRIVE_SYS_ID_QUASISTATIC_REVERSE,
    DRIVE_SYS_ID_DYNAMIC_FORWARD,
    DRIVE_SYS_ID_DYNAMIC_REVERSE
  }

  private class Auto {
    AutoName name;
    SequentialCommandGroup command;

    private Auto(AutoName name, SequentialCommandGroup command) {
      this.name = name;
      this.command = command;
    }
  }

  private List<Auto> autos;
  private AutoName defaultAuto = AutoName.DO_NOTHING;

  public AutonomousSelector(
      Drive drive,
      Hood hood,
      Turret turret,
      Shooter shooter,
      VisionObjectDetection visionObjectDetection,
      Intake intake) {
    if (Constants.enableCharacterizationAutos) {
      autos =
          List.of(
              new Auto(AutoName.DO_NOTHING, new DoNothing(hood)),
              new Auto(
                  AutoName.DRIVE_WHEEL_RADIUS_CHARACTERIZATION,
                  new SequentialCommandGroup(
                      Commands.race(
                          DriveCommands.wheelRadiusCharacterization(drive),
                          Commands.waitSeconds(60)))),
              new Auto(
                  AutoName.DRIVE_SIMPLE_FF_CHARACTERIZATION,
                  new SequentialCommandGroup(DriveCommands.feedforwardCharacterization(drive))),
              new Auto(
                  AutoName.DRIVE_SYS_ID_QUASISTATIC_FORWARD,
                  new SequentialCommandGroup(
                      Commands.race(
                          drive.sysIdQuasistatic(SysIdRoutine.Direction.FORWARD),
                          Commands.waitSeconds(6)))),
              new Auto(
                  AutoName.DRIVE_SYS_ID_QUASISTATIC_REVERSE,
                  new SequentialCommandGroup(
                      Commands.race(
                          drive.sysIdQuasistatic(SysIdRoutine.Direction.REVERSE),
                          Commands.waitSeconds(6)))),
              new Auto(
                  AutoName.DRIVE_SYS_ID_DYNAMIC_FORWARD,
                  new SequentialCommandGroup(
                      Commands.race(
                          drive.sysIdDynamic(SysIdRoutine.Direction.FORWARD),
                          Commands.waitSeconds(6)))),
              new Auto(
                  AutoName.DRIVE_SYS_ID_DYNAMIC_REVERSE,
                  new SequentialCommandGroup(
                      Commands.race(
                          drive.sysIdDynamic(SysIdRoutine.Direction.REVERSE),
                          Commands.waitSeconds(6)))));
    } else {
      autos =
          List.of(
              new Auto(AutoName.DO_NOTHING, new DoNothing(hood)),
              new Auto(AutoName.R_2_SWEEP, new R2Sweep(drive, intake, shooter, autoStartDelay)),
              new Auto(AutoName.L_2_SWEEP, new L2Sweep(drive, intake, shooter, autoStartDelay)),
              new Auto(
                  AutoName.L_2_SWEEP_DEPOT,
                  new L2SweepDepot(drive, intake, shooter, autoStartDelay)),
              new Auto(
                  AutoName.C_START_TO_DEPOT,
                  new CenterStartToDepot(drive, intake, shooter, autoStartDelay)),
              new Auto(
                  AutoName.C_PRELOAD, new CenterPreload(drive, intake, shooter, autoStartDelay)),
              new Auto(
                  AutoName.R_SECOND_SHALLOW_BUMP,
                  new RSecondShallowBump(drive, intake, shooter, autoStartDelay)),
              new Auto(
                  AutoName.R_SECOND_SHALLOW_BUMP_DEPOT,
                  new RSecondShallowBumpDepot(drive, intake, shooter, autoStartDelay)),
              new Auto(
                  AutoName.R_SECOND_SHALLOW_TRENCH,
                  new RSecondShallowTrench(drive, intake, shooter, autoStartDelay)),
              new Auto(
                  AutoName.R_SECOND_SHALLOW_TRENCH_DEPOT,
                  new RSecondShallowTrenchDepot(drive, intake, shooter, autoStartDelay)),
              new Auto(
                  AutoName.L_SECOND_SHALLOW_BUMP,
                  new LSecondShallowBump(drive, intake, shooter, autoStartDelay)),
              new Auto(
                  AutoName.L_SECOND_SHALLOW_TRENCH,
                  new LSecondShallowTrench(drive, intake, shooter, autoStartDelay)),
              new Auto(
                  AutoName.R_SINGLE_SWEEP,
                  new RSingleSweep(drive, intake, shooter, autoStartDelay)),
              new Auto(
                  AutoName.L_SINGLE_SWEEP,
                  new LSingleSweep(drive, intake, shooter, autoStartDelay)));
    }

    for (Auto nextAuto : autos) {
      if (nextAuto.name == defaultAuto) {
        autonomousSelector.addDefault(nextAuto.name.toString(), nextAuto.command);
      } else {
        autonomousSelector.add(nextAuto.name.toString(), nextAuto.command);
      }
    }
  }

  public Command get() {
    if (Constants.currentMode == Mode.SIM) {
      for (Auto nextAuto : autos) {
        if (nextAuto.name == Simulator.getAutoScenario()) {
          Logger.recordOutput("AutoName", Simulator.getAutoScenario());
          return nextAuto.command;
        }
      }
      System.out.println("Simulated auto " + Simulator.getAutoScenario() + " not found");
      System.exit(1);
    }
    return autonomousSelector.getSelected();
  }
}
