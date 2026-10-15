package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.generated.TunerConstants;
import org.junit.jupiter.api.Test;

class DrivetrainPowerModeTest {
    @Test
    void automaticallyEnablesOnlyOnceAndLeavesLaterChangesToTheDriver() throws Exception {
        assertTrue(HAL.initialize(500, 0));
        RoboRioSim.setVInVoltage(12.0);
        try (var drivetrain = TunerConstants.createDrivetrain()) {
            drivetrain.periodic();
            assertMode("Normal");
            for (var module : drivetrain.getModules()) {
                setSupplyLimits(module.getDriveMotor());
                setSupplyLimits(module.getSteerMotor());
            }
            assertLimits(drivetrain, 120, 60);

            drivetrain.toggleLowPowerMode();
            drivetrain.periodic();
            assertEnabledTopic(false); // Enabling has been requested but not yet confirmed.
            awaitMode(drivetrain, "Low power");
            assertLimits(drivetrain, 60, 40);
            drivetrain.toggleLowPowerMode();
            drivetrain.periodic();
            assertEnabledTopic(true); // Keep the applied mode until normal limits are confirmed.
            awaitMode(drivetrain, "Normal");
            assertLimits(drivetrain, 120, 60);

            // A second press while an update is running must win after that update completes.
            drivetrain.toggleLowPowerMode();
            drivetrain.periodic();
            drivetrain.toggleLowPowerMode();
            awaitMode(drivetrain, "Normal");
            assertLimits(drivetrain, 120, 60);

            setBatteryVoltage(drivetrain, 7.01);
            assertMode("Normal");
            setBatteryVoltage(drivetrain, 7.0);
            awaitMode(drivetrain, "Low power");
            assertLimits(drivetrain, 60, 40);

            // Voltage recovery must never automatically exit low power.
            setBatteryVoltage(drivetrain, 12.0);
            assertMode("Low power");

            // Manual exit remains effective for repeated samples at/below 7 V.
            setBatteryVoltage(drivetrain, 6.5);
            drivetrain.toggleLowPowerMode();
            awaitMode(drivetrain, "Normal");
            for (int i = 0; i < 20; i++) {
                drivetrain.periodic();
                assertMode("Normal");
            }
            assertLimits(drivetrain, 120, 60);

            // Neither small rebounds nor full recovery rearm the one-time trigger.
            for (double volts : new double[] {7.49, 6.8, 7.5, 6.9, 12.0, 7.0, 6.5}) {
                setBatteryVoltage(drivetrain, volts);
                assertMode("Normal");
            }
            assertLimits(drivetrain, 120, 60);

            // Start still allows the driver to enter and exit low power after the trigger is used.
            drivetrain.toggleLowPowerMode();
            awaitMode(drivetrain, "Low power");
            assertLimits(drivetrain, 60, 40);
            setBatteryVoltage(drivetrain, 12.0);
            assertMode("Low power");
            setBatteryVoltage(drivetrain, 7.0);
            assertMode("Low power");
            drivetrain.toggleLowPowerMode();
            awaitMode(drivetrain, "Normal");
            assertLimits(drivetrain, 120, 60);
        } finally {
            RoboRioSim.setVInVoltage(12.0);
        }
    }

    private static void setBatteryVoltage(CommandSwerveDrivetrain drivetrain, double volts) {
        RoboRioSim.setVInVoltage(volts);
        drivetrain.periodic();
        assertEquals(volts, SmartDashboard.getNumber("Drivetrain/BatteryVoltage", -1), 1e-6);
    }

    private static void setSupplyLimits(TalonFX motor) throws InterruptedException {
        var limits = new CurrentLimitsConfigs();
        // The simulated CAN bus can still be starting when the first motor is read.
        long deadline = System.nanoTime() + 15_000_000_000L;
        var status = motor.getConfigurator().refresh(limits);
        while (!status.isOK() && System.nanoTime() < deadline) {
            Thread.sleep(20);
            status = motor.getConfigurator().refresh(limits);
        }
        assertTrue(status.isOK(), status.toString());
        limits.withSupplyCurrentLimit(37).withSupplyCurrentLimitEnable(true)
            .withSupplyCurrentLowerLimit(25).withSupplyCurrentLowerTime(0.6);
        assertTrue(motor.getConfigurator().apply(limits).isOK());
    }

    private static void assertLimits(CommandSwerveDrivetrain drivetrain, double drive, double steer) {
        assertEquals(4, drivetrain.getModules().length);
        for (var module : drivetrain.getModules()) {
            assertMotorLimits(module.getDriveMotor(), drive);
            assertMotorLimits(module.getSteerMotor(), steer);
        }
    }

    private static void assertMotorLimits(TalonFX motor, double statorAmps) {
        var limits = new CurrentLimitsConfigs();
        assertTrue(motor.getConfigurator().refresh(limits).isOK());
        assertEquals(statorAmps, limits.StatorCurrentLimit, 1e-6);
        assertTrue(limits.StatorCurrentLimitEnable);
        assertEquals(37, limits.SupplyCurrentLimit, 1e-6);
        assertTrue(limits.SupplyCurrentLimitEnable);
        assertEquals(25, limits.SupplyCurrentLowerLimit, 1e-6);
        assertEquals(0.6, limits.SupplyCurrentLowerTime, 1e-6);
    }

    private static void assertMode(String expected) {
        assertEquals(expected, SmartDashboard.getString("Drivetrain/PowerMode", ""));
        assertEquals(expected.equals("Low power"),
            SmartDashboard.getBoolean("Drivetrain/LowPowerModeRequested", false));
        assertEnabledTopic(expected.equals("Low power"));
    }

    private static void assertEnabledTopic(boolean expected) {
        var entry = NetworkTableInstance.getDefault()
            .getEntry("/SmartDashboard/Drivetrain/LowPowerModeEnabled");
        assertTrue(entry.exists());
        assertEquals(expected, entry.getBoolean(!expected));
    }

    private static void awaitMode(CommandSwerveDrivetrain drivetrain, String expected) throws Exception {
        long deadline = System.nanoTime() + 5_000_000_000L;
        do {
            drivetrain.periodic();
            if (expected.equals(SmartDashboard.getString("Drivetrain/PowerMode", ""))) {
                assertMode(expected);
                return;
            }
            Thread.sleep(10);
        } while (System.nanoTime() < deadline);
        assertMode(expected);
    }
}
