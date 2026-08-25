package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.Alliance;

import static org.wpilib.units.Units.Seconds;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command3.Command;

import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.UtilityCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedTunableNumber;

public class CenterPreload {
  private static PathPlannerPath path = Robot.C_To_Depot;
  private static Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
  private static Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();

      public static Command auto(Drive drive, Intake intake, Shooter shooter, LoggedTunableNumber autoStartDelay) {
        return Command.sequence(
          Command.noRequirements(coroutine -> {
            if (Robot.alliance == Alliance.BLUE) {
              drive.setPose(startPoseBlue);
            } else {
              drive.setPose(startPoseRed);
            }
          }).named("setStartPose"),
          Command.waitFor(Seconds.of(autoStartDelay.get())).named("waitForAutoStartDelay"),
          IntakeCommands.intake(intake),
          AutoBuilder.followPath(Robot.C_PRELOAD),
          ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake).withTimeout(Seconds.of(10.0))
        ).named("CENTER_PRELOAD");
      }

}
