package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelDeadlineGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.UtilityCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.wpilib.tunable.TunableDouble;

public class L2SweepDepot extends SequentialCommandGroup {
  private PathPlannerPath firstPath;

  public L2SweepDepot(
      Drive drive, Intake intake, Shooter shooter, TunableDouble autoStartDelay) {
    firstPath = Robot.L_2SWEEP_A;
    Pose2d startPoseBlue = firstPath.getStartingHolonomicPose().get();
    Pose2d startPoseRed = firstPath.flipPath().getStartingHolonomicPose().get();

    setName("L_2_SWEEP_DEPOT");
    addCommands(
        new InstantCommand(
            () -> {
              if (Robot.alliance == Alliance.BLUE) {
                drive.setPose(startPoseBlue);
              } else {
                drive.setPose(startPoseRed);
              }
            }),
        new UtilityCommands.WaitSupplierCommand(autoStartDelay),
        IntakeCommands.intake(intake),
        AutoBuilder.followPath(firstPath),
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(Robot.L_2SWEEP_B),
            ShooterCommands.idle(shooter, intake, 15.0, 40.0, 95.0),
            ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec)),
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(Robot.L_2SWEEP_CG),
            ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake),
            IntakeCommands.autoSmoosh(
                intake,
                Constants.Autonomous.twoSweepSmooshDelayFirstPass,
                Constants.Autonomous.twoSweepShootTimeFirstPass)),
        IntakeCommands.intake(intake),
        new WaitUntilCommand(() -> shooter.isHoodLowered()),
        AutoBuilder.followPath(Robot.L_2SWEEP_DE),
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(Robot.L_2SWEEP_F),
            ShooterCommands.idle(shooter, intake, 14.0, 40.0, 95.0),
            ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec)),
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(Robot.L_2SWEEP_DEPOT),
            ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake)),
        ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake));
  }
}
