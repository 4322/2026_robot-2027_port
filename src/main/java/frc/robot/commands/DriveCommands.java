// Copyright (c) 2021-2026 Littleton Robotics
// http://github.com/Mechanical-Advantage
//
// Use of this source code is governed by a BSD
// license that can be found in the LICENSE file
// at the root directory of this project.

package frc.robot.commands;

import org.wpilib.math.util.MathUtil;
import org.wpilib.math.controller.ProfiledPIDController;
import org.wpilib.math.filter.SlewRateLimiter;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.geometry.Transform2d;
import org.wpilib.math.geometry.Translation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.trajectory.TrapezoidProfile;
import org.wpilib.math.util.Units;
import org.wpilib.driverstation.DriverStation;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.Alliance;
import org.wpilib.system.Timer;
import org.wpilib.command3.Command;

import frc.robot.Robot;
import frc.robot.constants.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.LoggedTunableNumber;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import static org.wpilib.units.Units.Seconds;

public class DriveCommands {
    private static final double DEADBAND = 0.0;
    private static final double FF_START_DELAY = 2.0; // Secs
    private static final double FF_RAMP_RATE = 0.1; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private static final LoggedTunableNumber rotateKp = new LoggedTunableNumber("DriveCommands/rotateKp", 3.0);
    private static final LoggedTunableNumber rotateKd = new LoggedTunableNumber("DriveCommands/rotateKd", 0.0);
    private static final LoggedTunableNumber rotateMaxVelocity = new LoggedTunableNumber(
            "DriveCommands/rotateMaxVelocity", 8.0);
    private static final LoggedTunableNumber rotateMaxAcceleration = new LoggedTunableNumber(
            "DriveCommands/rotateMaxAcceleration", 20.0);

    private DriveCommands() {
    }

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(Translation2d.kZero, linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, Rotation2d.kZero))
                .getTranslation();
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and
     * angular velocities).
     */
    public static Command joystickDrive(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return drive.runRepeatedly(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Cube rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisVelocities speeds = new ChassisVelocities(
                            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                            omega * drive.getMaxAngularSpeedRadPerSec());
                    boolean isFlipped = MatchState.getAlliance().isPresent() &&
                            MatchState.getAlliance().get() == Alliance.RED;
                    drive.runVelocity(
                            speeds.toRobotRelative(
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                })
                .named("joystickDrive");
    }

    public static Command joystickDriveWhileShooting(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            BooleanSupplier isScoring) {
        return drive.runRepeatedly(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                            ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Cube rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    double linearVelocityScaling = drive.getMaxLinearSpeedMetersPerSec();
                    double angularVelocityScaling = drive.getMaxAngularSpeedRadPerSec();

                    if (isScoring.getAsBoolean()) {
                        linearVelocityScaling *= Constants.Drive.maxLinearSpeedPercentShooting;
                        angularVelocityScaling *= Constants.Drive.maxAngularSpeedPercentShooting;
                    } else {
                        linearVelocityScaling *= Constants.Drive.maxLinearSpeedPercentPassing;
                        angularVelocityScaling *= Constants.Drive.maxAngularSpeedPercentPassing;
                    }

                    ChassisVelocities speeds = new ChassisVelocities(
                            linearVelocity.getX() * linearVelocityScaling,
                            linearVelocity.getY() * linearVelocityScaling,
                            omega * angularVelocityScaling);
                    boolean isFlipped = MatchState.getAlliance().isPresent()
                            && MatchState.getAlliance().get() == Alliance.RED;
                    drive.runVelocity(
                            speeds.toRobotRelative(
                                    isFlipped
                                            ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : drive.getRotation()));
                })
                .named("joystickDriveWhileShooting");
    }

    /**
     * Field relative drive command using joystick for linear control and PID for
     * angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target,
     * or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier) {
        return joystickDriveAtAngle(drive, xSupplier, ySupplier, rotationSupplier, Translation2d.kZero);
    }

    public static Command joystickDriveAtAngle(
            Drive drive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            Supplier<Rotation2d> rotationSupplier,
            Translation2d centerOfRotation) {

        // Create PID controller
        ProfiledPIDController angleController = new ProfiledPIDController(0, 0, 0,
                new TrapezoidProfile.Constraints(0, 0));
        angleController.enableContinuousInput(-Math.PI, Math.PI);

        // Construct command
        return drive.run(
                coroutine -> {
                    // Reset PID controller when command starts
                    angleController.reset(drive.getRotation().getRadians());
                    angleController.setPID(rotateKp.get(), 0.0, rotateKd.get());
                    angleController.setConstraints(
                            new TrapezoidProfile.Constraints(
                                    rotateMaxVelocity.get(), rotateMaxAcceleration.get()));
                    while (true) {
                        // Get linear velocity
                        Translation2d linearVelocity = getLinearVelocityFromJoysticks(xSupplier.getAsDouble(),
                                ySupplier.getAsDouble());

                        // Calculate angular speed
                        double omega = angleController.calculate(
                                drive.getRotation().getRadians(), rotationSupplier.get().getRadians());

                        // Convert to field relative speeds & send command
                        ChassisVelocities speeds = new ChassisVelocities(
                                linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
                                linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
                                omega);
                        boolean isFlipped = MatchState.getAlliance().isPresent()
                                && MatchState.getAlliance().get() == Alliance.RED;
                        ChassisVelocities robotRelative = speeds.toRobotRelative( // TODO see if this is correct usage of ChassisVelocities/ChassisVelocities
                                isFlipped
                                        ? drive.getRotation().plus(new Rotation2d(Math.PI))
                                        : drive.getRotation());
                        ChassisVelocities corrected = rotateAboutShooter(robotRelative, centerOfRotation);
                        drive.runVelocity(corrected);
                        coroutine.yield();
                    }

                }).named("joystickDriveAtAngle");
    }

    /**
     * Rotates robot about shooter location instead of center. This works by adding
     * a translational
     * velocity that cancels the shooter movement due to rotation.
     */
    public static ChassisVelocities rotateAboutShooter(
            ChassisVelocities robotRelativeSpeeds, Translation2d shooterOffset) {

        // If we're rotating with omega, the shooter moves tangentially
        // We need to add translation to keep shooter stationary
        double omega = robotRelativeSpeeds.omega;

        // Velocity of shooter due to rotation about center: v = omega x r
        // In 2D: vx = -omega * y, vy = omega * x
        double shooterVxFromRotation = -omega * shooterOffset.getY();
        double shooterVyFromRotation = omega * shooterOffset.getX();

        // Subtract this from robot velocity to make shooter the rotation center
        return new ChassisVelocities(
                robotRelativeSpeeds.vx - shooterVxFromRotation,
                robotRelativeSpeeds.vy - shooterVyFromRotation,
                omega);
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>
     * This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Command.sequence(
                // Reset data
                Command.noRequirements(
                        coroutine -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        })
                        .named("feedforwardCharacterization - resetData"),
                // Allow modules to orient
                drive.runRepeatedly(
                        () -> {
                            drive.runCharacterization(0.0);
                        })
                        .named("feedforwardCharacterization - orientModules")
                        .withTimeout(Seconds.of(FF_START_DELAY)),

                // Start timer
                Command.noRequirements(coroutine -> {
                    timer.restart();
                })
                        .named("feedforwardCharacterization - startTimer"),

                // Accelerate and gather data
                drive.runRepeatedly(
                        () -> {
                            double voltage = timer.get() * FF_RAMP_RATE;
                            drive.runCharacterization(voltage);
                            velocitySamples.add(drive.getFFCharacterizationVelocity());
                            voltageSamples.add(voltage);
                        })

                        // When cancelled, calculate and print results
                        .whenCanceled(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS = (sumY * sumX2 - sumX * sumXY) / (n * sumX2 - sumX * sumX);
                                    double kV = (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println("********** Drive FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                })
                        .named("feedforwardCharacterization - gatherData"))
                .withAutomaticName();
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Command.parallel(
                // Drive control sequence
                Command.sequence(
                        // Reset acceleration limiter
                        Command.noRequirements(
                                coroutine -> {
                                    limiter.reset(0.0);
                                }).named("wheelRadiusCharacterization - resetLimiter"),

                        // Turn in place, accelerating up to full speed
                        drive.run(
                                coroutine -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisVelocities(0.0, 0.0, speed));
                                })
                                .named("wheelRadiusCharacterization - accelerate"))
                        .withAutomaticName(),

                // Measurement sequence
                Command.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Command.waitFor(Seconds.of(1.0)).named("wheelRadiusCharacterization - waitBeforeMeasurement"),
                        // Record starting measurement
                        Command.noRequirements(
                                coroutine -> {
                                    state.positions = drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                })
                                .named("wheelRadiusCharacterization - recordStartingMeasurement"),

                        // Update gyro delta
                        drive.run(
                                coroutine -> {
                                    while (true) {
                                        var rotation = drive.getRotation();
                                        state.gyroDelta += Math.abs(rotation.minus(state.lastAngle).getRadians());
                                        state.lastAngle = rotation;
                                        coroutine.yield();
                                    }
                                })
                            .whenCanceled(() -> {
                                double[] positions = drive.getWheelRadiusCharacterizationPositions();
                                    double wheelDelta = 0.0;
                                    for (int i = 0; i < 4; i++) {
                                        wheelDelta += Math.abs(positions[i] - state.positions[i]) / 4.0;
                                    }
                                    double wheelRadius = (state.gyroDelta * Drive.DRIVE_BASE_RADIUS) / wheelDelta;

                                    NumberFormat formatter = new DecimalFormat("#0.000");
                                    System.out.println(
                                            "********** Wheel Radius Characterization Results **********");
                                    System.out.println(
                                            "\tWheel Delta: " + formatter.format(wheelDelta) + " radians");
                                    System.out.println(
                                            "\tGyro Delta: " + formatter.format(state.gyroDelta) + " radians");
                                    System.out.println(
                                            "\tWheel Radius: "
                                                    + formatter.format(wheelRadius)
                                                    + " meters, "
                                                    + formatter.format(Units.metersToInches(wheelRadius))
                                                    + " inches");
                            })
                            .named("feedforwardCharacterization - updateGyroDelta")
                // When cancelled, calculate and print results
                ).withAutomaticName()
            ).withAutomaticName();
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = Rotation2d.kZero;
        double gyroDelta = 0.0;
    }
}
