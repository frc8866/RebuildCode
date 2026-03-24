// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    


    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Shooter shooter = new Shooter();
    public final Hood hood = new Hood();
    public final Intake intake = new Intake();
    public final Superstructure superstructure = new Superstructure(drivetrain, shooter, hood, intake);
    private static final String kShooterCameraName = "shooter";
    private static final String kLeftCameraName = "left1";
    private static final String kRightCameraName = "right1";

    private final Transform3d robotToShooterCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(2), Units.inchesToMeters(0.003), Units.inchesToMeters(17)),
        new Rotation3d(0.0, Math.toRadians(-15), Math.toRadians(180)));

    private final Transform3d robotToLeftCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(-13), Units.inchesToMeters(0), Units.inchesToMeters(20)),
        new Rotation3d(0.0, Math.toRadians(10), Math.toRadians(38)));

    private final Transform3d robotToRightCam = new Transform3d(
        new Translation3d(Units.inchesToMeters(10), Units.inchesToMeters(0), Units.inchesToMeters(20)),
        new Rotation3d(0.0, Math.toRadians(10), Math.toRadians(-39)));

    public final Vision vision = new Vision(
        drivetrain::addVisionMeasurement,
        new VisionIOPhotonVision(kShooterCameraName, robotToShooterCam),
        new VisionIOPhotonVision(kLeftCameraName, robotToLeftCam),
        new VisionIOPhotonVision(kRightCameraName, robotToRightCam));

    /* Path follower */
    private final LoggedDashboardChooser<Command> autoChooser;

    public RobotContainer() {
        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());

        NamedCommands.registerCommand("shoot",
            Commands.sequence(
                superstructure.setShooting(),
                Commands.defer(
                    () -> drivetrain.aimAt(superstructure.getHubHeading(), () -> superstructure.isPassing())
                        .withTimeout(10),
                    java.util.Set.of(drivetrain)),
                superstructure.goIn(true),
                superstructure.setDriving()));
        NamedCommands.registerCommand("accshoot", superstructure.setShooting());
        NamedCommands.registerCommand("intake", superstructure.setIntake());
        NamedCommands.registerCommand("drive",
            Commands.sequence(superstructure.setDriving(), superstructure.goIn(true)));
        NamedCommands.registerCommand("faceforward",
            Commands.defer(
                () -> drivetrain.aimAt(Rotation2d.fromDegrees(0)).withTimeout(3),
                java.util.Set.of(drivetrain)));

        configureBindings();

        PathPlannerLogging.setLogActivePathCallback((activePath) -> {
            superstructure.setTrajectory(activePath.toArray(new Pose2d[0]));
        });
        PathPlannerLogging.setLogTargetPoseCallback((targetPose) -> {
            superstructure.setTrajectorySetpoint(targetPose);
        });

        // Warmup PathPlanner to avoid Java pauses
        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
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

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on left bumper press.
        joystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.rightTrigger().whileTrue(drivetrain.aimAt(superstructure.getHubHeading(), () -> superstructure.isPassing()));
        joystick.rightTrigger().onTrue(superstructure.setShooting())
            .onFalse(superstructure.setDriving());

        joystick.leftTrigger().onTrue(superstructure.setIntake())
            .onFalse(superstructure.setDriving());

        joystick.leftBumper().onTrue(superstructure.goIn(true)).onFalse(superstructure.goIn(false));
        joystick.rightBumper().onTrue(superstructure.spin(true)).onFalse(superstructure.spin(false));

        joystick.y().onTrue(superstructure.setDriving());

        joystick.start().and(joystick.y().negate()).and(joystick.x().negate())
            .onTrue(superstructure.setReverseIntake())
            .onFalse(superstructure.setDriving());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        Command selectedAuto = autoChooser.get();
    if (selectedAuto instanceof PathPlannerAuto pathPlannerAuto
        && pathPlannerAuto.getStartingPose() != null) {
      return Commands.sequence(
          AutoBuilder.resetOdom(pathPlannerAuto.getStartingPose()),
          
          selectedAuto.asProxy());
    }
    return selectedAuto;
    
    }
}
