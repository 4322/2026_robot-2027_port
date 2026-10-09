package frc.robot.autonomous.modes;

import org.wpilib.command2.ConditionalCommand;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.ParallelCommandGroup;
import org.wpilib.command2.ParallelDeadlineGroup;
import org.wpilib.command2.SequentialCommandGroup;
import org.wpilib.command2.WaitCommand;
import org.wpilib.command2.WaitUntilCommand;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.geometry.Pose2d;

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
import frc.robot.util.LoggedTunableNumber;

public class SmartThreeSweep extends SequentialCommandGroup {

    public SmartThreeSweep(Drive drive, Intake intake, Shooter shooter, LoggedTunableNumber autoStartDelay, PathPlannerPath firstPath, PathPlannerPath secondPath, PathPlannerPath thirdPath, boolean isRightSide) { 
        Pose2d startPoseBlue = firstPath.getStartingHolonomicPose().get();
        Pose2d startPoseRed = firstPath.flipPath().getStartingHolonomicPose().get();

        setName(isRightSide ? "R" : "L" + firstPath.name + "-" + secondPath.name + "-" + thirdPath.name);

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
                new ParallelCommandGroup( // Makes sure the path actually finishes before we go to next command
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> shooter.isFuelDetected()), // Wait until we start shooting, then wait until we stop
                        new WaitUntilCommand(() -> !shooter.isShooting())
                    ),
                    AutoBuilder.followPath(isRightSide ? Robot.RShoot : Robot.LShoot)
                ),
                ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake),
                IntakeCommands.autoSmoosh(
                    intake,
                    Constants.Autonomous.twoSweepSmooshDelayFirstPass,
                    Constants.Autonomous.twoSweepShootTimeFirstPass)),
            new WaitUntilCommand(() -> shooter.isHoodLowered()),
            AutoBuilder.followPath(isRightSide ? Robot.RReturn : Robot.LReturn),
            AutoBuilder.followPath(secondPath),
            new ParallelDeadlineGroup(
                AutoBuilder.followPath(isRightSide ? Robot.RSweepBump : Robot.LSweepBump),
                ShooterCommands.idle(shooter, intake, 15.0, 40.0, 95.0),
                ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec)),
            new ParallelDeadlineGroup(
                new ParallelCommandGroup( // Makes sure the path actually finishes before we go to next command
                    new SequentialCommandGroup(
                        new WaitUntilCommand(() -> shooter.isFuelDetected()), // Wait until we start shooting, then wait until we stop
                        new WaitUntilCommand(() -> !shooter.isShooting())
                    ),
                    AutoBuilder.followPath(isRightSide ? Robot.RShoot : Robot.LShoot)
                ),
                ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake),
                IntakeCommands.autoSmoosh(
                    intake,
                    Constants.Autonomous.twoSweepSmooshDelayFirstPass,
                    Constants.Autonomous.twoSweepShootTimeFirstPass)),
            new WaitUntilCommand(() -> shooter.isHoodLowered()),
            AutoBuilder.followPath(isRightSide ? Robot.RReturn : Robot.LReturn),
            new ConditionalCommand(
                new SequentialCommandGroup(
                    AutoBuilder.followPath(thirdPath),
                    new ParallelDeadlineGroup(
                        AutoBuilder.followPath(isRightSide ? Robot.RSweepBump : Robot.LSweepBump),
                        ShooterCommands.idle(shooter, intake, 15.0, 40.0, 95.0),
                        ShooterCommands.autoUnjam(shooter, Constants.Autonomous.unjamTimeSec)),
                    ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake).withTimeout(10) // Timeout just to make sure it doesn't cause any issues
                ),
                AutoBuilder.followPath(isRightSide ? Robot.RSweepOut : Robot.LSweepOut),
                () -> MatchState.getMatchTime() > Constants.Autonomous.thirdSweepTimeMin
            )
        );
    }
}
