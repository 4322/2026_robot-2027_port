// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.
package frc.robot;

import org.wpilib.driverstation.GenericHID;
import org.wpilib.smartdashboard.Field2d;
import org.wpilib.smartdashboard.SmartDashboard;
import org.wpilib.command2.Command;
import frc.robot.autonomous.AutonomousSelector;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.IntakeCommands;
import frc.robot.commands.ShooterCommands;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Simulator;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOBoron;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOTalonFX;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.deployer.Deployer;
import frc.robot.subsystems.intake.deployer.DeployerIO;
import frc.robot.subsystems.intake.deployer.DeployerIOSim;
import frc.robot.subsystems.intake.deployer.DeployerIOTalonFX;
import frc.robot.subsystems.intake.rollers.Rollers;
import frc.robot.subsystems.intake.rollers.RollersIO;
import frc.robot.subsystems.intake.rollers.RollersIOSim;
import frc.robot.subsystems.intake.rollers.RollersIOTalonFX;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.led.LEDIO;
import frc.robot.subsystems.led.LEDIOCANdle;
import frc.robot.subsystems.led.LEDIOSim;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.flywheel.Flywheel;
import frc.robot.subsystems.shooter.flywheel.FlywheelIO;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOSim;
import frc.robot.subsystems.shooter.flywheel.FlywheelIOTalonFx;
import frc.robot.subsystems.shooter.hood.Hood;
import frc.robot.subsystems.shooter.hood.HoodIO;
import frc.robot.subsystems.shooter.hood.HoodIOSim;
import frc.robot.subsystems.shooter.hood.HoodIOTalonFx;
import frc.robot.subsystems.shooter.spindexer.Spindexer;
import frc.robot.subsystems.shooter.spindexer.SpindexerIO;
import frc.robot.subsystems.shooter.spindexer.SpindexerIOSim;
import frc.robot.subsystems.shooter.spindexer.SpindexerIOTalonFx;
import frc.robot.subsystems.shooter.tunnel.Tunnel;
import frc.robot.subsystems.shooter.tunnel.TunnelIO;
import frc.robot.subsystems.shooter.tunnel.TunnelIOSim;
import frc.robot.subsystems.shooter.tunnel.TunnelIOTalonFx;
import frc.robot.subsystems.shooter.turret.Turret;
import frc.robot.subsystems.shooter.turret.TurretIO;
import frc.robot.subsystems.shooter.turret.TurretIOSim;
import frc.robot.subsystems.shooter.turret.TurretIOTalonFx;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPose;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPoseIO;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPoseIOPhoton;
import frc.robot.subsystems.vision.visionGlobalPose.VisionGlobalPoseIOSim;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetection;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetectionIO;
import frc.robot.subsystems.vision.visionObjectDetection.VisionObjectDetectionIOPhoton;
import org.wpilib.command2.button.CommandNiDsXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // Subsystems
    public static VisionGlobalPose visionGlobalPose;

    private static VisionObjectDetection visionObjectDetection;

    public static Shooter shooter;

    private static Flywheel flywheel;

    private static Hood hood;

    private static Spindexer spindexer;

    private static Tunnel tunnel;

    private static Turret turret;

    public static Intake intake;

    private static LED led;

    private static Rollers rollers;

    private static Deployer deployer;

    public static Drive drive;

    public static Simulator simulator;

    public static AutonomousSelector autonomousSelector;

    private static Field2d field;

    // Controller
    public static final CommandNiDsXboxController controller = new CommandNiDsXboxController(0);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        switch(Constants.currentMode) {
            case REAL ->
                {
                    // Real robot, instantiate hardware IO implementations
                    // ModuleIOTalonFX is intended for modules with TalonFX drive, TalonFX turn, and
                    // a CANcoder
                    drive = Constants.driveMode == Constants.SubsystemMode.DISABLED ? new Drive(new GyroIO() {
                    }, new ModuleIO() {
                    }, new ModuleIO() {
                    }, new ModuleIO() {
                    }, new ModuleIO() {
                    }) : new Drive(new GyroIOBoron(), new ModuleIOTalonFX(TunerConstants.FrontLeft), new ModuleIOTalonFX(TunerConstants.FrontRight), new ModuleIOTalonFX(TunerConstants.BackLeft), new ModuleIOTalonFX(TunerConstants.BackRight));
                    visionGlobalPose = Constants.visionGlobalPose == Constants.SubsystemMode.DISABLED ? new VisionGlobalPose(drive, new VisionGlobalPoseIO() {
                    }, new VisionGlobalPoseIO() {
                    }, new VisionGlobalPoseIO() {
                    }, new VisionGlobalPoseIO() {
                    }) : new VisionGlobalPose(drive, Constants.frontRightCameraEnable ? new VisionGlobalPoseIOPhoton(Constants.VisionGlobalPose.frontRightName, Constants.VisionGlobalPose.frontRightTransform) : new VisionGlobalPoseIO() {
                    }, Constants.frontLeftCameraEnable ? new VisionGlobalPoseIOPhoton(Constants.VisionGlobalPose.frontLeftName, Constants.VisionGlobalPose.frontLeftTransform) : new VisionGlobalPoseIO() {
                    }, Constants.backRightCameraEnable ? new VisionGlobalPoseIOPhoton(Constants.VisionGlobalPose.backRightName, Constants.VisionGlobalPose.backRightTransform) : new VisionGlobalPoseIO() {
                    }, Constants.backLeftCameraEnable ? new VisionGlobalPoseIOPhoton(Constants.VisionGlobalPose.backLeftName, Constants.VisionGlobalPose.backLeftTransform) : new VisionGlobalPoseIO() {
                    });
                    visionObjectDetection = Constants.visionObjectDetection == Constants.SubsystemMode.DISABLED ? new VisionObjectDetection(drive, new VisionObjectDetectionIO() {
                    }) : new VisionObjectDetection(drive, new VisionObjectDetectionIOPhoton());
                    led = new LED(Constants.ledMode == Constants.SubsystemMode.DISABLED ? new LEDIO() {
                    } : new LEDIOCANdle(), drive);
                    flywheel = Constants.flywheelMode == Constants.SubsystemMode.DISABLED ? new Flywheel(new FlywheelIO() {
                    }) : new Flywheel(new FlywheelIOTalonFx());
                    hood = Constants.hoodMode == Constants.SubsystemMode.DISABLED ? new Hood(new HoodIO() {
                    }) : new Hood(new HoodIOTalonFx());
                    spindexer = Constants.spindexerMode == Constants.SubsystemMode.DISABLED ? new Spindexer(new SpindexerIO() {
                    }) : new Spindexer(new SpindexerIOTalonFx());
                    tunnel = Constants.tunnelMode == Constants.SubsystemMode.DISABLED ? new Tunnel(new TunnelIO() {
                    }) : new Tunnel(new TunnelIOTalonFx());
                    turret = Constants.turretMode == Constants.SubsystemMode.DISABLED ? new Turret(new TurretIO() {
                    }) : new Turret(new TurretIOTalonFx());
                    shooter = new Shooter(flywheel, hood, spindexer, tunnel, turret, visionGlobalPose, drive);
                    rollers = Constants.rollerMode == Constants.SubsystemMode.DISABLED ? new Rollers(new RollersIO() {
                    }) : new Rollers(new RollersIOTalonFX());
                    deployer = Constants.deployerMode == Constants.SubsystemMode.DISABLED ? new Deployer(new DeployerIO() {
                    }) : new Deployer(new DeployerIOTalonFX());
                    intake = new Intake(deployer, rollers);
                }
            case SIM ->
                {
                    // Sim robot, instantiate physics sim IO implementations
                    drive = Constants.driveMode == Constants.SubsystemMode.DISABLED ? new Drive(new GyroIO() {
                    }, new ModuleIO() {
                    }, new ModuleIO() {
                    }, new ModuleIO() {
                    }, new ModuleIO() {
                    }) : new Drive(new GyroIO() {
                    }, new ModuleIOSim(TunerConstants.FrontLeft), new ModuleIOSim(TunerConstants.FrontRight), new ModuleIOSim(TunerConstants.BackLeft), new ModuleIOSim(TunerConstants.BackRight));
                    visionGlobalPose = Constants.visionGlobalPose == Constants.SubsystemMode.DISABLED ? new VisionGlobalPose(drive, new VisionGlobalPoseIO() {
                    }, new VisionGlobalPoseIO() {
                    }, new VisionGlobalPoseIO() {
                    }, new VisionGlobalPoseIO() {
                    }) : new VisionGlobalPose(drive, new VisionGlobalPoseIOSim(Constants.VisionGlobalPose.frontRightName, drive::getRobotPose, Constants.VisionGlobalPose.frontRightTransform), new VisionGlobalPoseIOSim(Constants.VisionGlobalPose.frontLeftName, drive::getRobotPose, Constants.VisionGlobalPose.frontLeftTransform), new VisionGlobalPoseIOSim(Constants.VisionGlobalPose.backRightName, drive::getRobotPose, Constants.VisionGlobalPose.backRightTransform), new VisionGlobalPoseIOSim(Constants.VisionGlobalPose.backLeftName, drive::getRobotPose, Constants.VisionGlobalPose.backLeftTransform));
                    visionObjectDetection = Constants.visionObjectDetection == Constants.SubsystemMode.DISABLED ? new VisionObjectDetection(drive, new VisionObjectDetectionIO() {
                    }) : new VisionObjectDetection(drive, // TODO make sim for this
                    new VisionObjectDetectionIO() {
                    });
                    led = new LED(Constants.ledMode == Constants.SubsystemMode.DISABLED ? new LEDIO() {
                    } : new LEDIOSim(), drive);
                    flywheel = Constants.flywheelMode == Constants.SubsystemMode.DISABLED ? new Flywheel(new FlywheelIO() {
                    }) : new Flywheel(new FlywheelIOSim());
                    hood = Constants.hoodMode == Constants.SubsystemMode.DISABLED ? new Hood(new HoodIO() {
                    }) : new Hood(new HoodIOSim());
                    spindexer = Constants.spindexerMode == Constants.SubsystemMode.DISABLED ? new Spindexer(new SpindexerIO() {
                    }) : new Spindexer(new SpindexerIOSim());
                    tunnel = Constants.tunnelMode == Constants.SubsystemMode.DISABLED ? new Tunnel(new TunnelIO() {
                    }) : new Tunnel(new TunnelIOSim());
                    turret = Constants.turretMode == Constants.SubsystemMode.DISABLED ? new Turret(new TurretIO() {
                    }) : new Turret(new TurretIOSim());
                    shooter = new Shooter(flywheel, hood, spindexer, tunnel, turret, visionGlobalPose, drive);
                    deployer = Constants.deployerMode == Constants.SubsystemMode.DISABLED ? new Deployer(new DeployerIO() {
                    }) : new Deployer(new DeployerIOSim());
                    rollers = Constants.rollerMode == Constants.SubsystemMode.DISABLED ? new Rollers(new RollersIO() {
                    }) : new Rollers(new RollersIOSim());
                    intake = new Intake(deployer, rollers);
                    simulator = new Simulator();
                }
            default ->
                {
                    // Replayed robot, disable IO implementations
                    drive = new Drive(new GyroIO() {
                    }, new ModuleIO() {
                    }, new ModuleIO() {
                    }, new ModuleIO() {
                    }, new ModuleIO() {
                    });
                    visionGlobalPose = new VisionGlobalPose(drive, new VisionGlobalPoseIO() {
                    }, new VisionGlobalPoseIO() {
                    }, new VisionGlobalPoseIO() {
                    }, new VisionGlobalPoseIO() {
                    });
                    led = new LED(new LEDIO() {
                    }, drive);
                    visionObjectDetection = new VisionObjectDetection(drive, new VisionObjectDetectionIO() {
                    });
                    flywheel = new Flywheel(new FlywheelIO() {
                    });
                    hood = new Hood(new HoodIO() {
                    });
                    spindexer = new Spindexer(new SpindexerIO() {
                    });
                    tunnel = new Tunnel(new TunnelIO() {
                    });
                    turret = new Turret(new TurretIO() {
                    });
                    shooter = new Shooter(flywheel, hood, spindexer, tunnel, turret, visionGlobalPose, drive);
                    rollers = new Rollers(new RollersIO() {
                    });
                    deployer = new Deployer(new DeployerIO() {
                    });
                    intake = new Intake(deployer, rollers);
                }
        }
        // Configure the button bindings
        configureButtonBindings();
        if (Constants.enableElasticOdometry) {
            field = new Field2d();
            SmartDashboard.putData("Field", field);
        }
        if (Constants.debugZoneAreas) {
            FieldConstants.plotZones();
        }
    }

    public static Field2d getField() {
        return Constants.enableElasticOdometry ? field : new Field2d();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * org.wpilib.driverstation.Joystick} or {@link XboxController}), and then passing it to a {@link
     * org.wpilib.command2.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        // Default command, normal field-relative drive
        drive.setDefaultCommand(DriveCommands.joystickDrive(drive, () -> -controller.getLeftY(), () -> -controller.getLeftX(), () -> -controller.getRightX()));
        /*
    Intake - Left Bumper Toggle (Driver)
    Trench Override Hood - Left Trigger while held (Driver)
    Unjam Shooter - B while held (Driver)
    Shoot - Right Trigger while held (Driver)
    Fixed Shooting - 'a' while held (Driver)
    Smoosh - 'y' on true (Driver)
    Turret Unjam - right bumper while held (Driver)
    Intake Eject - 'x' on true (Driver)
    */
        controller.b().whileTrue(ShooterCommands.unjam(shooter));
        controller.leftTrigger().whileTrue(ShooterCommands.trenchOverride(shooter));
        if (Constants.turretLocked) {
            controller.rightTrigger().whileTrue(ShooterCommands.aimAndShoot(shooter, drive, intake));
        } else {
            controller.rightTrigger().whileTrue(ShooterCommands.autoShoot(shooter, drive, intake));
            controller.a().whileTrue(ShooterCommands.fixedShoot(shooter, drive, intake));
        }
        controller.leftBumper().onTrue(IntakeCommands.toggleIntake(intake, controller));
        controller.x().onTrue(IntakeCommands.eject(intake)).onFalse(IntakeCommands.toggleOff(intake));
        controller.y().onTrue(IntakeCommands.smoosh(intake)).onFalse(IntakeCommands.toggleOff(intake));
        controller.rightBumper().onTrue(ShooterCommands.turretUnjamOverride(shooter, true));
        controller.rightBumper().onFalse(ShooterCommands.turretUnjamOverride(shooter, false));
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return autonomousSelector.get();
    }

    public void configureAutonomousSelector() {
        autonomousSelector = new AutonomousSelector(drive, hood, turret, shooter, visionObjectDetection, intake);
    }

    public void setBrakeMode(boolean brake) {
        spindexer.enableBrakeMode(brake);
        turret.setBrakeMode(brake);
        deployer.setBrakeMode(brake);
        hood.setBrakeMode(brake);
    }
}
