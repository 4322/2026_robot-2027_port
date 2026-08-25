package frc.robot.autonomous.modes;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.command2.InstantCommand;
import org.wpilib.command2.SequentialCommandGroup;
import frc.robot.Robot;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.commands.UtilityCommands;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.LoggedTunableNumber;
import org.wpilib.driverstation.Alliance;

public class CenterPreload extends SequentialCommandGroup {

    public CenterPreload(Drive drive, Intake intake, Shooter shooter, LoggedTunableNumber autoStartDelay) {
        PathPlannerPath path = Robot.C_To_Depot;
        Pose2d startPoseBlue = path.getStartingHolonomicPose().get();
        Pose2d startPoseRed = path.flipPath().getStartingHolonomicPose().get();
        setName("CENTER_PRELOAD");
        addCommands(new InstantCommand(() -> {
            if (Robot.alliance == Alliance.BLUE) {
                drive.setPose(startPoseBlue);
            } else {
                drive.setPose(startPoseRed);
            }
        }), new UtilityCommands.WaitSupplierCommand(autoStartDelay), IntakeCommands.intake(intake), AutoBuilder.followPath(Robot.C_PRELOAD), ShooterCommands.autoShootNoAreaCheck(shooter, drive, intake).withTimeout(10.0));
    }
}
