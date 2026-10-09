// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;


import frc.robot.commands.FeedforwardCharacterizationCommand;
import frc.robot.commands.WheelRadiusCharacterizationCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Superstructure.SuperstructureCurrentState;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.vision.Vision;
import frc.robot.subsystems.vision.VisionIOPhotonVision;
import frc.robot.subsystems.led.LED;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.util.FuelSim;




public class RobotContainer {
    private static final double kMotorTooHotThresholdC = 54.4444444444;
    private static final double kDriveDeadband = 0.2;
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity


    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * kDriveDeadband)
            .withRotationalDeadband(MaxAngularRate * kDriveDeadband)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric()
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    


    private final Telemetry logger = new Telemetry(MaxSpeed);

    private boolean autoRotateFlip = false;
    private boolean autoIsTier2 = false;

    private final CommandXboxController joystick = new CommandXboxController(0);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final Shooter shooter = new Shooter();
    public final Hood hood = new Hood();
    public final Intake intake = new Intake();
    public final LED led = new LED(intake);
    public final Superstructure superstructure = new Superstructure(drivetrain, shooter, hood, intake);
    private static final String kShooterCameraName = "shooter";
    private static final String kLeftCameraName = "left";
    private static final String kRightCameraName = "right";

// trying fuelsim
    public FuelSim fuelSim;


    private final NetworkTable motorHealthTable =
        NetworkTableInstance.getDefault().getTable("MotorHealth");
    private final List<MonitoredMotor> monitoredMotors = List.of(
        new MonitoredMotor("Front Left Drive", new TalonFX(8, TunerConstants.kCANBus.getName())),
        new MonitoredMotor("Front Left Steer", new TalonFX(3, TunerConstants.kCANBus.getName())),
        new MonitoredMotor("Front Right Drive", new TalonFX(9, TunerConstants.kCANBus.getName())),
        new MonitoredMotor("Front Right Steer", new TalonFX(4, TunerConstants.kCANBus.getName())),
        new MonitoredMotor("Back Left Drive", new TalonFX(6, TunerConstants.kCANBus.getName())),
        new MonitoredMotor("Back Left Steer", new TalonFX(1, TunerConstants.kCANBus.getName())),
        new MonitoredMotor("Back Right Drive", new TalonFX(7, TunerConstants.kCANBus.getName())),
        new MonitoredMotor("Back Right Steer", new TalonFX(2, TunerConstants.kCANBus.getName())),
        new MonitoredMotor("Shooter Leader", new TalonFX(1)),
        new MonitoredMotor("Shooter Follower", new TalonFX(2)),
        new MonitoredMotor("Spindexer Spinner", new TalonFX(18)),
        new MonitoredMotor("Spindexer Wheel", new TalonFX(20)),
        new MonitoredMotor("Intake Arm", new TalonFX(11)),
        new MonitoredMotor("Intake Wheels", new TalonFX(16)),
        new MonitoredMotor("Hood", new TalonFX(17))
        
    );


    private final Transform3d robotToShooterCam = new Transform3d( // luma
        new Translation3d(Units.inchesToMeters(2), Units.inchesToMeters(0.003), Units.inchesToMeters(17)),
        new Rotation3d(0.0, Math.toRadians(-15), Math.toRadians(180)));

    private final Transform3d robotToLeftCam = new Transform3d( // opposite snakeskin
        new Translation3d(Units.inchesToMeters(-12.675550), Units.inchesToMeters(-11.210814), Units.inchesToMeters(20.874904)),
        new Rotation3d(Math.toRadians(121.267527578), Math.toRadians(331.761804914-12), Math.toRadians(228.509579)));

    private final Transform3d robotToRightCam = new Transform3d( //snakeskin
        new Translation3d(Units.inchesToMeters(12.675550), Units.inchesToMeters(-11.210814), Units.inchesToMeters(20.874904)),
        new Rotation3d(Math.toRadians(121.267527578), Math.toRadians(331.761804914-12), Math.toRadians(131.490421)));

    Pose3d ShooterCamPose = new Pose3d().transformBy(robotToShooterCam); // luma
    Pose3d LeftCamPose = new Pose3d().transformBy(robotToLeftCam); // opposite snakeskin
    Pose3d RigthCamPose = new Pose3d().transformBy(robotToRightCam); //snakeskin


        // only luma
    public final Vision vision = new Vision(
        drivetrain::addVisionMeasurement,
        cameraIndex -> {
            if (DriverStation.isAutonomousEnabled()) {
                if (cameraIndex == 0) { // shooter cam only during shooting states
                    return superstructure.currentState == Superstructure.SuperstructureCurrentState.SHOOTING
                        || superstructure.currentState == Superstructure.SuperstructureCurrentState.PRE_SHOOT;
                }
                return false; // left/right cams off in auto
            }
            return true; // teleop: all cameras accepted
        },
        new VisionIOPhotonVision(kShooterCameraName, robotToShooterCam));

        // luma & orange pi
    // public final Vision vision = new Vision(
    //     drivetrain::addVisionMeasurement,
    //     cameraIndex -> {
    //         if (DriverStation.isAutonomousEnabled()) {
    //             if (cameraIndex == 0) { // shooter cam only during shooting states
    //                 return superstructure.currentState == Superstructure.SuperstructureCurrentState.SHOOTING
    //                     || superstructure.currentState == Superstructure.SuperstructureCurrentState.PRE_SHOOT;
    //             }
    //             return false; // left/right cams off in auto
    //         }
    //         return true; // teleop: all cameras accepted
    //     },
    //     new VisionIOPhotonVision(kShooterCameraName, robotToShooterCam), // Camera0
    //     new VisionIOPhotonVision(kLeftCameraName, robotToLeftCam), // Camera1
    //     new VisionIOPhotonVision(kRightCameraName, robotToRightCam)); // Camera2
        
    
        // only orange pi
    // public final Vision vision = new Vision(
    //     drivetrain::addVisionMeasurement,
    //     cameraIndex -> {
    //         if (DriverStation.isAutonomousEnabled()) {
    //             if (cameraIndex == 0) { // shooter cam only during shooting states
    //                 return superstructure.currentState == Superstructure.SuperstructureCurrentState.SHOOTING
    //                     || superstructure.currentState == Superstructure.SuperstructureCurrentState.PRE_SHOOT;
    //             }
    //             return false; // left/right cams off in auto
    //         }
    //         return true; // teleop: all cameras accepted
    //     }, 
    //     new VisionIOPhotonVision(kLeftCameraName, robotToLeftCam),  
    //     new VisionIOPhotonVision(kRightCameraName, robotToRightCam));


    /* Path follower */
    private final LoggedDashboardChooser<Command> autoChooser;

    private record MonitoredMotor(String name, TalonFX motor) {}

    public RobotContainer() {
        Logger.recordOutput("Opposite SnakeSkin Left Cam Pose", LeftCamPose);
        Logger.recordOutput("SnakeSkin Right Cam Pose", RigthCamPose);
        Logger.recordOutput("Shooter Cam Pose", ShooterCamPose);

        NamedCommands.registerCommand("shoot",
            Commands.deadline(
                Commands.waitSeconds(5).andThen(Commands.runOnce(() -> {
                    if (DriverStation.isAutonomous() && autoIsTier2) {
                        drivetrain.resetPose(vision.getLatestAcceptedPose().orElse(drivetrain.getPose()));
                    }
                })).andThen(superstructure.setDriving()),
                Commands.sequence(
                    superstructure.setIntake(),
                    Commands.waitSeconds(0.2),
                    superstructure.setSlowIntakeIn(),
                    superstructure.setShooting(),
                    Commands.defer(
                        () -> drivetrain.aimAt(
                            () -> {
                                boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
                                double deg = (isBlue ? 180 : 0) + (autoRotateFlip ? 180 : 0);
                                return superstructure.getHubHeading().plus(Rotation2d.fromDegrees(deg));
                            },
                            () -> 0.0,
                            () -> 0.0)
                            .repeatedly().withTimeout(2),
                        java.util.Set.of(drivetrain)),
                    superstructure.goIn(false)),
                Commands.waitSeconds(1).andThen(superstructure.setForceShoot(false))));

        NamedCommands.registerCommand("longerShoot",
            Commands.deadline(
                Commands.waitSeconds(7).andThen(Commands.runOnce(() -> {
                    if (DriverStation.isAutonomous() && autoIsTier2) {
                        drivetrain.resetPose(vision.getLatestAcceptedPose().orElse(drivetrain.getPose()));
                    }
                })).andThen(superstructure.setDriving()),
                Commands.sequence(
                    superstructure.setIntake(),
                    Commands.waitSeconds(0.2),
                    superstructure.setSlowIntakeIn(),
                    superstructure.setShooting(),
                    Commands.defer(
                        () -> drivetrain.aimAt(
                            () -> {
                                boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
                                double deg = (isBlue ? 180 : 0) + (autoRotateFlip ? 180 : 0);
                                return superstructure.getHubHeading().plus(Rotation2d.fromDegrees(deg));
                            },
                            () -> 0.0,
                            () -> 0.0)
                            .repeatedly().withTimeout(2),
                        java.util.Set.of(drivetrain)),
                    superstructure.goIn(false)),
                Commands.waitSeconds(1).andThen(superstructure.setForceShoot(false))));

                
        NamedCommands.registerCommand("accshoot", Commands.sequence(superstructure.setShooting(), Commands.waitSeconds(5)));


        NamedCommands.registerCommand("intake", superstructure.setIntake());
        NamedCommands.registerCommand("drive",
            Commands.sequence(superstructure.setDriving(), superstructure.goIn(true)));
        NamedCommands.registerCommand("faceforward",
            Commands.defer(
                () -> drivetrain.aimAt(Rotation2d.fromDegrees(0)).withTimeout(3),
                java.util.Set.of(drivetrain)));

        autoChooser = new LoggedDashboardChooser<>("Auto Chooser", AutoBuilder.buildAutoChooser());
        autoChooser.addOption("Feedforward Characterization", new FeedforwardCharacterizationCommand(drivetrain));
        autoChooser.addOption("Wheel Radius Characterization", new WheelRadiusCharacterizationCommand(drivetrain));

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
        
        //regular drive
         drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );
        
        // uhhhh idt this works yet
        // drivetrain.setDefaultCommand(
        //     drivetrain.applyRequest(() -> {
        //         ChassisSpeeds speeds = new ChassisSpeeds(
        //             -joystick.getLeftY() * MaxSpeed,
        //             -joystick.getLeftX() * MaxSpeed,
        //             -joystick.getRightX() * MaxAngularRate
        //         );

        //         speeds = superstructure.applyShootOnMoveSpeedLock(speeds);

        //         return drive.withVelocityX(speeds.vxMetersPerSecond)
        //             .withVelocityY(speeds.vyMetersPerSecond)
        //             .withRotationalRate(speeds.omegaRadiansPerSecond);
        //     })
        // );


        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        joystick.b().onTrue(superstructure.setSlowIntakeIn()).onFalse(superstructure.setDriving());

        joystick.povUp().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0.5).withVelocityY(0))
        );
        joystick.povDown().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(-0.5).withVelocityY(0))
        );

        joystick.povLeft().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(0.5))
        );
        joystick.povRight().whileTrue(drivetrain.applyRequest(() ->
            forwardStraight.withVelocityX(0).withVelocityY(-0.5))
        );


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        // joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        // joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        // joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        // joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // Reset the field-centric heading on a press.
        joystick.a().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        joystick.x().onTrue(Commands.runOnce(() -> autoRotateFlip = !autoRotateFlip));

            // varible speeds sotm- TODO need to test
        // joystick.rightTrigger().and(joystick.leftTrigger().negate()).and(new Trigger(superstructure::isManual).negate())
        //     .whileTrue(drivetrain.aimAtWithXLock(
        //         () -> {
        //             boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
        //             double deg = (isBlue ? 180 : 0) + (autoRotateFlip ? 180 : 0);
        //             return superstructure.getHubHeading().plus(Rotation2d.fromDegrees(deg));
        //         },
        //         this::getShootingDriveSpeeds,
        //         () -> false));
                //!superstructure.isPassing() to turn sotm off

            //shuttling only, sotm off
            joystick.rightTrigger().and(joystick.leftTrigger().negate()).and(new Trigger(superstructure::isManual).negate())
                .whileTrue(drivetrain.aimAtWithXLock(
                    () -> {
                        boolean isBlue = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Blue;
                        double deg = (isBlue ? 180 : 0) + (autoRotateFlip ? 180 : 0);
                        return superstructure.getHubHeading().plus(Rotation2d.fromDegrees(deg));
                    },
                    () -> -MathUtil.clamp(joystick.getLeftY() * 10, -1, 1) * 0, //0.4 is the max with optimizations
                    () -> -MathUtil.clamp(joystick.getLeftX() * 10, -1, 1) * 0, //0.4 is the max with optimizations
                    () -> !superstructure.isPassing()));
                //false to turn sotm on

        joystick.rightTrigger().and(joystick.leftTrigger().negate()).onTrue(
                Commands.either(superstructure.setPassing(), superstructure.setShooting(), superstructure::isPassing))
            .onFalse(superstructure.setDriving());

        // Keep intaking (including reverse spindexer) until another action changes the state.
        //joystick.leftTrigger().and(joystick.rightTrigger().negate()).onTrue(superstructure.setIntake());

           joystick.leftTrigger().and(joystick.rightTrigger().negate()).onTrue(superstructure.setIntake())
            .onFalse(superstructure.setDriving());


        joystick.leftTrigger().and(joystick.rightTrigger())
            .onTrue(superstructure.setIntakeHold())
            .onFalse(superstructure.setDriving());

        joystick.leftBumper().onTrue(superstructure.goIn(true)).onFalse(superstructure.goIn(false));

        joystick.y().onTrue(superstructure.setDriving());

        joystick.rightBumper()
            .onTrue(superstructure.setReverseIntake())
            .onFalse(superstructure.setDriving());

        drivetrain.registerTelemetry(logger::telemeterize);

            }
    public Command getAutonomousCommand() {
        /* Run the path selected from the auto chooser */
        Command selectedAuto = autoChooser.get();
        autoIsTier2 = (selectedAuto instanceof PathPlannerAuto ppa) && ppa.getName().contains("Teir 2");
    if (selectedAuto instanceof PathPlannerAuto pathPlannerAuto
        && pathPlannerAuto.getStartingPose() != null) {
      return Commands.sequence(
          AutoBuilder.resetOdom(pathPlannerAuto.getStartingPose()),

          selectedAuto.asProxy());
    }
    return selectedAuto;
    
    }

    public void updateMotorHealthDashboard() {
        StringBuilder disconnectedMotors = new StringBuilder();
        StringBuilder hotMotors = new StringBuilder();
        List<String> disconnectedMotorNames = new ArrayList<>();
        List<String> hotMotorNames = new ArrayList<>();
        boolean allMotorsConnected = true;
        boolean allMotorsCool = true;

        for (MonitoredMotor monitoredMotor : monitoredMotors) {
            boolean connected = monitoredMotor.motor().isConnected();
            double motorTempC = monitoredMotor.motor().getDeviceTemp().getValueAsDouble();
            boolean tooHot = motorTempC > kMotorTooHotThresholdC;
            String motorKey = monitoredMotor.name().replace(" ", "");
            NetworkTable motorTable = motorHealthTable.getSubTable(motorKey);

            motorTable.getEntry("Name").setString(monitoredMotor.name());
            motorTable.getEntry("Connected").setBoolean(connected);
            motorTable.getEntry("TempC").setDouble(motorTempC);
            motorTable.getEntry("TooHot").setBoolean(tooHot);

            if (!connected) {
                allMotorsConnected = false;
                disconnectedMotorNames.add(monitoredMotor.name());
                appendMotorName(disconnectedMotors, monitoredMotor.name());
            }

            if (tooHot) {
                allMotorsCool = false;
                String hotMotorText = monitoredMotor.name() + String.format(" (%.2f C)", motorTempC);
                hotMotorNames.add(hotMotorText);
                appendMotorName(hotMotors, hotMotorText);
            }
        }

        String disconnectedText = disconnectedMotors.length() == 0 ? "None" : disconnectedMotors.toString();
        String hotText = hotMotors.length() == 0 ? "None" : hotMotors.toString();
        String overallStatus;

        if (allMotorsConnected && allMotorsCool) {
            overallStatus = "All motors connected and below " + kMotorTooHotThresholdC + " C";
        } else if (!allMotorsConnected && !allMotorsCool) {
            overallStatus = "Disconnected: " + disconnectedText + " | Too hot: " + hotText;
        } else if (!allMotorsConnected) {
            overallStatus = "Disconnected: " + disconnectedText;
        } else {
            overallStatus = "Too hot: " + hotText;
        }

        motorHealthTable.getEntry("AllConnected").setBoolean(allMotorsConnected);
        motorHealthTable.getEntry("AllCool").setBoolean(allMotorsCool);
        motorHealthTable.getEntry("Healthy").setBoolean(allMotorsConnected && allMotorsCool);
        motorHealthTable.getEntry("DisconnectedMotors").setString(disconnectedText);
        motorHealthTable.getEntry("DisconnectedMotorsList")
            .setStringArray(disconnectedMotorNames.toArray(String[]::new));
        motorHealthTable.getEntry("HotMotors").setString(hotText);
        motorHealthTable.getEntry("HotMotorsList")
            .setStringArray(hotMotorNames.toArray(String[]::new));
        motorHealthTable.getEntry("OverallStatus").setString(overallStatus);
        motorHealthTable.getEntry("TooHotThresholdC").setDouble(kMotorTooHotThresholdC);
    }

    // private ChassisSpeeds getShootingDriveSpeeds() {
    //     ChassisSpeeds requestedSpeeds;
    //     if (superstructure.isShootOnMoveSpeedLimitActive()) {
    //         requestedSpeeds = new ChassisSpeeds(
    //             -MathUtil.applyDeadband(joystick.getLeftY(), 0.2) * MaxSpeed,
    //             -MathUtil.applyDeadband(joystick.getLeftX(), 0.2) * MaxSpeed,
    //             0.0);
    //     } else {
    //         // Retain the existing fixed-speed controls and deadband outside moving hub shots.
    //         double vx = -MathUtil.clamp(joystick.getLeftY() * 10, -1, 1) * 0.2;
    //         double vy = -MathUtil.clamp(joystick.getLeftX() * 10, -1, 1) * 0.2;
    //         requestedSpeeds = Math.hypot(vx, vy) < 0.2
    //             ? new ChassisSpeeds()
    //             : new ChassisSpeeds(vx, vy, 0.0);
    //     }
    //     return superstructure.applyShootOnMoveDistanceSpeedLimit(requestedSpeeds);
    // }

    private static final double kPassingMaxSpeed = 0.1; // m/s — total vector magnitude cap while passing

    private edu.wpi.first.math.geometry.Translation2d getPassingVelocity() {
        double vx = MathUtil.applyDeadband(-joystick.getLeftY(), 0.2) * MaxSpeed;
        double vy = MathUtil.applyDeadband(-joystick.getLeftX(), 0.2) * MaxSpeed;
        double mag = Math.hypot(vx, vy);
        if (mag > kPassingMaxSpeed) {
            double scale = kPassingMaxSpeed / mag;
            vx *= scale;
            vy *= scale;
        }
        return new edu.wpi.first.math.geometry.Translation2d(vx, vy);
    }

    private static void appendMotorName(StringBuilder builder, String motorName) {
        if (builder.length() > 0) {
            builder.append(", ");
        }
        builder.append(motorName);
    }
}
