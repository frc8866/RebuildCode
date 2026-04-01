package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Slowly ramps drive motor voltage and records velocity to compute kS and kV
 * feedforward gains for the drive motors.
 *
 * Run as an auto routine. Once enabled, the robot will sit still for
 * START_DELAY_SECS, then drive forward with increasing voltage.
 * Cancel (disable) when the robot reaches full speed or hits a wall.
 *
 * Results are printed to the RoboRIO console and SmartDashboard under
 * "FF Char/kS" and "FF Char/kV". Update driveGains in TunerConstants with
 * these values.
 */
public class FeedforwardCharacterizationCommand extends Command {
    private static final double START_DELAY_SECS = 2.0;
    private static final double RAMP_VOLTS_PER_SEC = 0.1;
    private static final double MAX_VOLTS = 6.0;

    private final CommandSwerveDrivetrain drivetrain;
    private final SwerveRequest.SysIdSwerveTranslation translationRequest =
        new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SwerveDriveBrake brakeRequest =
        new SwerveRequest.SwerveDriveBrake();

    private final List<Double> velocities = new ArrayList<>();
    private final List<Double> appliedVolts = new ArrayList<>();
    private double startTime;

    public FeedforwardCharacterizationCommand(CommandSwerveDrivetrain drivetrain) {
        this.drivetrain = drivetrain;
        addRequirements(drivetrain);
    }

    @Override
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
        velocities.clear();
        appliedVolts.clear();
        System.out.println("[FF Char] Starting — robot will drive forward in " + START_DELAY_SECS + "s");
    }

    @Override
    public void execute() {
        double elapsed = Timer.getFPGATimestamp() - startTime;

        if (elapsed < START_DELAY_SECS) {
            drivetrain.setControl(brakeRequest);
            return;
        }

        double volts = Math.min((elapsed - START_DELAY_SECS) * RAMP_VOLTS_PER_SEC, MAX_VOLTS);
        drivetrain.setControl(translationRequest.withVolts(Volts.of(volts)));

        // Average absolute speed across all modules (m/s)
        var states = drivetrain.getState().ModuleStates;
        double avgVelocity = 0;
        for (var state : states) {
            avgVelocity += Math.abs(state.speedMetersPerSecond);
        }
        avgVelocity /= states.length;

        velocities.add(avgVelocity);
        appliedVolts.add(volts);

        SmartDashboard.putNumber("FF Char/Current Volts", volts);
        SmartDashboard.putNumber("FF Char/Current Velocity (m-s)", avgVelocity);
    }

    @Override
    public void end(boolean interrupted) {
        drivetrain.setControl(brakeRequest);

        if (velocities.size() < 10) {
            System.out.println("[FF Char] Not enough data — run longer next time.");
            return;
        }

        // Linear regression:  voltage = kS + kV * velocity
        int n = velocities.size();
        double sumX = 0, sumY = 0, sumXY = 0, sumX2 = 0;
        for (int i = 0; i < n; i++) {
            double x = velocities.get(i);
            double y = appliedVolts.get(i);
            sumX  += x;
            sumY  += y;
            sumXY += x * y;
            sumX2 += x * x;
        }
        double denom = n * sumX2 - sumX * sumX;
        if (Math.abs(denom) < 1e-9) {
            System.out.println("[FF Char] Degenerate data — could not compute gains.");
            return;
        }
        double kV = (n * sumXY - sumX * sumY) / denom;
        double kS = (sumY - kV * sumX) / n;

        System.out.println("========= Feedforward Characterization Results =========");
        System.out.printf("  kS = %.5f V%n",     kS);
        System.out.printf("  kV = %.5f V*s/m%n", kV);
        System.out.println("  Put these in TunerConstants driveGains:");
        System.out.printf("    .withKS(%.5f).withKV(%.5f)%n", kS, kV);
        System.out.println("========================================================");

        SmartDashboard.putNumber("FF Char/kS", kS);
        SmartDashboard.putNumber("FF Char/kV", kV);
    }

    @Override
    public boolean isFinished() {
        return false; // cancel (disable) to stop and see results
    }
}
