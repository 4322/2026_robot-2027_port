package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.InstantCommand;
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

public class LSingleSweep extends SequentialCommandGroup {
  PathPlannerPath firstPath;

  public LSingleSweep(
      Drive drive, Intake intake, Shooter shooter, TunableDouble autoStartDelay) {
    firstPath = Robot.L_SINGLE_SWEEP_A;
    Pose2d startPoseBlue = firstPath.getStartingHolonomicPose().get();
    Pose2d startPoseRed = firstPath.flipPath().getStartingHolonomicPose().get();

    setName("L_SINGLE_SWEEP");

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
        IntakeCommands.autoIntake(intake),
        AutoBuilder.followPath(firstPath),
        AutoBuilder.followPath(Robot.L_SINGLE_SWEEP_B),
        ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec),
        ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake));
  }
}
