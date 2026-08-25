package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.UtilityCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedTunableNumber;

public class LSingleSweepDepot extends SequentialCommandGroup {
  PathPlannerPath firstPath;

  public LSingleSweepDepot(
      Drive drive, Intake intake, Shooter shooter, LoggedTunableNumber autoStartDelay) {
    firstPath = Robot.L_SINGLE_SWEEP_A;
    Pose2d startPoseBlue = firstPath.getStartingHolonomicPose().get();
    Pose2d startPoseRed = firstPath.flipPath().getStartingHolonomicPose().get();

    setName("L_SINGLE_SWEEP_DEPOT");

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
        new ParallelCommandGroup(
            AutoBuilder.followPath(Robot.L_SINGLE_SWEEP_B_DEPOT),
            ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake)));
  }
}
