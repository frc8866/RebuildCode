package frc.robot.commands;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Spins the robot in place and compares gyro rotation to drive-encoder travel
 * to compute the actual wheel radius.
 *
 * Run as an auto routine. Let the robot spin for at least 3–5 full rotations,
 * then cancel (disable). The final wheel radius is printed to the console and
 * shown on SmartDashboard under "Wheel Radius Char/Final Wheel Radius (in)".
 * Update kWheelRadius in TunerConstants with this value.
 *
 * Module positions:  FL (+11, +11), FR (+11, -11), BL (-11, +11), BR (-11, -11) inches
 * Drive base radius: sqrt(11² + 11²) in = ~15.556 in = 0.39513 m
 */
public class WheelRadiusCharacterizationCommand extends Command {
    // sqrt(11² + 11²) inches, converted to metres
    private static final double DRIVE_BASE_RADIUS_M =
        Units.inchesToMeters(Math.sqrt(11.0 * 11.0 + 11.0 * 11.0));
    private static final double DRIVE_GEAR_RATIO = 6.026785714285714;
    private static final double SPIN_SPEED_RAD_PER_SEC = 1.0; // rad/s — slow enough to be safe
    private static final double SETTLE_SECS = 1.0; // let steer settle before recording

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.RobotCentric spinRequest = new SwerveRequest.RobotCentric()
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withVelocityX(0).withVelocityY(0);
    private final SwerveRequest.SwerveDriveBrake brakeRequest =
        new SwerveRequest.SwerveDriveBrake();

    private double[] startPositions; // drive motor rotations at start (4 modules)
    private double lastHeadingRad;
    private double accumulatedAngleRad;
    private double settleStartTime;

    public WheelRadiusCharacterizationCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        settleStartTime = Timer.getFPGATimestamp();
        lastHeadingRad = drivetrain.getPose().getRotation().getRadians();
        accumulatedAngleRad = 0;

        var modules = drivetrain.getModules();
        startPositions = new double[modules.length];
        for (int i = 0; i < modules.length; i++) {
            startPositions[i] = modules[i].getDriveMotor().getPosition().getValueAsDouble();
        }

        System.out.println("[Wheel Radius Char] Starting — robot will spin in place.");
        System.out.printf("  Drive base radius: %.5f m (%.4f in)%n",
            DRIVE_BASE_RADIUS_M, Units.metersToInches(DRIVE_BASE_RADIUS_M));
    }

    @Override
    public void execute() {
        drivetrain.setControl(spinRequest.withRotationalRate(SPIN_SPEED_RAD_PER_SEC));

        // Track accumulated heading (handles wrap-around correctly)
        double currentHeadingRad = drivetrain.getPose().getRotation().getRadians();
        accumulatedAngleRad += MathUtil.angleModulus(currentHeadingRad - lastHeadingRad);
        lastHeadingRad = currentHeadingRad;

        SmartDashboard.putNumber("Wheel Radius Char/Accumulated Angle (deg)",
            Math.toDegrees(Math.abs(accumulatedAngleRad)));

        // Don't compute until settled and we have meaningful rotation
        boolean settled = (Timer.getFPGATimestamp() - settleStartTime) > SETTLE_SECS;
        if (!settled || Math.abs(accumulatedAngleRad) < 0.2) return;

        double radius = computeRadius();
        SmartDashboard.putNumber("Wheel Radius Char/Live Wheel Radius (m)",  radius);
        SmartDashboard.putNumber("Wheel Radius Char/Live Wheel Radius (in)", Units.metersToInches(radius));
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brakeRequest);

        if (Math.abs(accumulatedAngleRad) < 0.2) {
            System.out.println("[Wheel Radius Char] Not enough rotation — spin longer next time.");
            return;
        }

        double radius = computeRadius();

        System.out.println("======= Wheel Radius Characterization Results =======");
        System.out.printf("  Wheel Radius  = %.5f m  (%.4f in)%n",
            radius, Units.metersToInches(radius));
        System.out.printf("  Total Rotation = %.2f deg (%.2f rotations)%n",
            Math.toDegrees(Math.abs(accumulatedAngleRad)),
            Math.abs(accumulatedAngleRad) / (2 * Math.PI));
        System.out.println("  Update TunerConstants:  kWheelRadius = Inches.of( <value above> )");
        System.out.println("=====================================================");

        SmartDashboard.putNumber("Wheel Radius Char/Final Wheel Radius (m)",  radius);
        SmartDashboard.putNumber("Wheel Radius Char/Final Wheel Radius (in)", Units.metersToInches(radius));
    }

    @Override
    public boolean isFinished() {
        return false; // cancel (disable) to stop and see results
    }

    /**
     * wheelRadius = (|gyroAngle| * driveBaseRadius) / (avgMotorRotations / gearRatio * 2π)
     */
    private double computeRadius() {
        var modules = drivetrain.getModules();
        double avgMotorRotations = 0;
        for (int i = 0; i < modules.length; i++) {
            double pos = modules[i].getDriveMotor().getPosition().getValueAsDouble();
            avgMotorRotations += Math.abs(pos - startPositions[i]);
        }
        avgMotorRotations /= modules.length;

        double wheelRotations = avgMotorRotations / DRIVE_GEAR_RATIO;
        return (Math.abs(accumulatedAngleRad) * DRIVE_BASE_RADIUS_M)
               / (wheelRotations * 2.0 * Math.PI);
    }
}
