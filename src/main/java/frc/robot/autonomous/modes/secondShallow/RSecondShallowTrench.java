package frc.robot.autonomous.modes.secondShallow;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelDeadlineGroup;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.UtilityCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.wpilib.tunable.TunableDouble;

public class RSecondShallowTrench extends SequentialCommandGroup {
  private PathPlannerPath firstPath;

  public RSecondShallowTrench(
      Drive drive, Intake intake, Shooter shooter, TunableDouble autoStartDelay) {
    firstPath = Robot.R_SECOND_SHALLOW_TRENCH;
    Pose2d startPoseBlue = firstPath.getStartingHolonomicPose().get();
    Pose2d startPoseRed = firstPath.flipPath().getStartingHolonomicPose().get();

    setName("R_SECOND_SHALLOW_TRENCH");
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
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(firstPath),
            ShooterCommands.idle(shooter, intake, 0.0, 40.0, null),
            ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec)),
        ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake));
  }
}
