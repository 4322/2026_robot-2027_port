package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.driverstation.Alliance;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.ParallelDeadlineGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.WaitUntilCommand;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.UtilityCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import org.wpilib.tunable.TunableDouble;
import org.littletonrobotics.junction.Logger;

public class CenterStartToDepot extends SequentialCommandGroup {
  public CenterStartToDepot(
      Drive drive, Intake intake, Shooter shooter, TunableDouble autoStartDelay) {
    PathPlannerPath path = Robot.C_To_Depot;
    Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
    Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();
    setName("C_Start_To_DEPOT");

    addCommands(
        new InstantCommand(() -> Logger.recordOutput("Autonomous/autoStarted", true)),
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
        new WaitUntilCommand(() -> intake.hasExtended()),
        new ParallelDeadlineGroup(
            AutoBuilder.followPath(Robot.C_To_Depot),
            new SequentialCommandGroup(
                new WaitCommand(1), ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake))),
        new ParallelCommandGroup(
            IntakeCommands.autoSmoosh(intake, 0, 5),
            ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake)));
  }
}
