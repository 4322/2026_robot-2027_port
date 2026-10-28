package frc.robot.autonomous.modes;

import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.ParallelDeadlineGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.WaitUntilCommand;
import org.wpilib.driverstation.Alliance;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.tunable.TunableDouble;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.UtilityCommands;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;

public class TwoSweep extends SequentialCommandGroup {

    public TwoSweep(Drive drive, Intake intake, Shooter shooter, TunableDouble autoStartDelay, PathPlannerPath firstPath, PathPlannerPath secondPath, boolean isRightSide) { 
        Pose2d startPoseBlue = firstPath.getStartingHolonomicPose().get();
        Pose2d startPoseRed = firstPath.flipPath().getStartingHolonomicPose().get();

        setName(isRightSide ? "R" : "L" + firstPath.name + "-" + secondPath.name);

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
            new ParallelCommandGroup(
                AutoBuilder.followPath(firstPath),
                new SequentialCommandGroup(
                    new WaitCommand(Constants.Autonomous.intakeDelay),
                    IntakeCommands.intake(intake)
                )
            ),
            new ParallelDeadlineGroup(
            AutoBuilder.followPath(isRightSide ? Robot.RSweepBump : Robot.LSweepBump),
            ShooterCommands.idle(shooter, intake, 15.0, 40.0, 95.0),
            ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec)),
            new ParallelDeadlineGroup(
            AutoBuilder.followPath(isRightSide ? Robot.RShoot : Robot.LShoot),
            ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake),
            IntakeCommands.autoSmoosh(
                intake,
                Constants.Autonomous.twoSweepSmooshDelayFirstPass,
                Constants.Autonomous.twoSweepSmooshTimeFirstPass)),
            new WaitUntilCommand(() -> shooter.isHoodLowered()),
            AutoBuilder.followPath(isRightSide ? Robot.RReturn : Robot.LReturn),
            AutoBuilder.followPath(secondPath),
            new ParallelDeadlineGroup(
            AutoBuilder.followPath(isRightSide ? Robot.RSweep2Bump : Robot.LSweep2Bump),
            ShooterCommands.idle(shooter, intake, 15.0, 40.0, 95.0),
            ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec)),
            new ParallelDeadlineGroup(
            AutoBuilder.followPath(isRightSide ? Robot.RShoot2 : Robot.LShoot2),
            ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake),
            IntakeCommands.autoSmoosh(
                intake,
                Constants.Autonomous.twoSweepSmooshDelaySecondPass,
                Constants.Autonomous.twoSweepSmooshTimeSecondPass)),
            new WaitUntilCommand(() -> shooter.isHoodLowered()),
            AutoBuilder.followPath(isRightSide ? Robot.RReturn : Robot.LReturn),
            AutoBuilder.followPath(isRightSide ? Robot.RSweepOut : Robot.LSweepOut)
        );
    }
}
