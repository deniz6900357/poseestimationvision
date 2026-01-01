// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.Velocity) // Use open-loop control for drive motors eskiden OpenLoopVoltage vardı
            .withSteerRequestType(com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType.MotionMagicExpo); // Advanced steer control
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
    public final VisionSubsystem vision = new VisionSubsystem();

    private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
        configureAutoBuilder();
        configureBindings();

        // Build auto chooser after AutoBuilder is configured
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }

    private void configureAutoBuilder() {
        try {
            RobotConfig config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> drivetrain.getState().Pose, // Pose2d supplier
                drivetrain::resetPose, // Pose2d consumer for resetting odometry
                () -> drivetrain.getKinematics().toChassisSpeeds(drivetrain.getState().ModuleStates), // ChassisSpeeds supplier
                (speeds, feedforwards) -> drivetrain.setControl(
                    new SwerveRequest.ApplyRobotSpeeds()
                        .withSpeeds(speeds)
                        .withDriveRequestType(DriveRequestType.Velocity)
                ), // ChassisSpeeds consumer
                new PPHolonomicDriveController(
                    // PID constants for translation
                    // These may need to be tuned for your robot
                    new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0),
                    // PID constants for rotation
                    new com.pathplanner.lib.config.PIDConstants(5.0, 0.0, 0.0)
                ),
                config,
                () -> false, // Should flip path for red alliance (set to false for now)
                drivetrain // Drivetrain requirements
            );
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.x().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and POV (D-pad).
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(() -> joystick.getHID().getPOV() == 0).whileTrue(drivetrain.sysIdDynamic(Direction.kForward)); // Back + D-pad Up
        joystick.back().and(() -> joystick.getHID().getPOV() == 180).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse)); // Back + D-pad Down
        joystick.start().and(() -> joystick.getHID().getPOV() == 0).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward)); // Start + D-pad Up
        joystick.start().and(() -> joystick.getHID().getPOV() == 180).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse)); // Start + D-pad Down

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        // Limelight vision aiming - Hold A button to aim and range at target
        joystick.a().whileTrue(
            drivetrain.applyRequest(() -> {
                if (vision.hasTarget()) {
                    // Use Limelight for aiming (rotation) and ranging (forward/back)
                    return drive
                        .withVelocityX(vision.getRangingForwardSpeed()) // Forward/backward based on ty
                        .withVelocityY(0) // No strafing during vision tracking
                        .withRotationalRate(vision.getAimingRotationSpeed()); // Rotation based on tx
                } else {
                    // No target - stop movement
                    return drive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0);
                }
            })
        );

        // Limelight strafe align - Hold Y button to align with target
        // Robot rotates to face target, strafes left/right to center, and ranges forward/back
        // All three movements happen simultaneously
        joystick.y().whileTrue(
            drivetrain.applyRequest(() -> {
                if (vision.hasTarget()) {
                    // Rotate to face target, strafe to center horizontally, range for distance
                    return drive
                        .withVelocityX(vision.getRangingForwardSpeed()) // Forward/backward based on ty
                        .withVelocityY(vision.getStrafeSpeed()) // Strafe left/right based on tx
                        .withRotationalRate(vision.getAimingRotationSpeed()); // Rotate to face target
                } else {
                    // No target - stop movement
                    return drive
                        .withVelocityX(0)
                        .withVelocityY(0)
                        .withRotationalRate(0);
                }
            })
        );

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
