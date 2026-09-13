// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

public class Robot extends LoggedRobot {
    private static final double kRecentBootWindowSeconds = 15.0;
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    private final NetworkTable roboRioHealthTable =
        NetworkTableInstance.getDefault().getTable("RoboRIOHealth");
    private final double robotBootTimestampSeconds = Timer.getFPGATimestamp();

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    private final boolean kUseLimelight = false;

    public Robot() {
        // Set up AdvantageKit logger
        if (isReal()) {
            Logger.addDataReceiver(new WPILOGWriter()); // Log to USB stick (/U/logs) or RIO
            Logger.addDataReceiver(new NT4Publisher()); // Publish to NetworkTables for live viewing
        } else {
            // Simulation: publish to NT4 for AdvantageScope
            Logger.addDataReceiver(new NT4Publisher());
        }
        Logger.start(); // Must be called before any subsystems are created

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        m_robotContainer.updateMotorHealthDashboard();
        updateRoboRioHealthTable();

        /*
         * This example of adding Limelight is very simple and may not be sufficient for on-field use.
         * Users typically need to provide a standard deviation that scales with the distance to target
         * and changes with number of tags available.
         *
         * This example is sufficient to show that vision integration is possible, though exact implementation
         * of how to use vision should be tuned per-robot and to the team's specification.
         */
        if (kUseLimelight) {
            var driveState = m_robotContainer.drivetrain.getState();
            double headingDeg = driveState.Pose.getRotation().getDegrees();
            double omegaRps = Units.radiansToRotations(driveState.Speeds.omegaRadiansPerSecond);

      
        }
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {
    }

    private void updateRoboRioHealthTable() {
        double uptimeSeconds = Timer.getFPGATimestamp() - robotBootTimestampSeconds;
        boolean brownedOutNow = RobotController.isBrownedOut();
        boolean rebootedRecently = uptimeSeconds < kRecentBootWindowSeconds;
        boolean systemHealthy =
            !brownedOutNow
                && RobotController.getEnabled3V3()
                && RobotController.getEnabled5V()
                && RobotController.getEnabled6V();

        roboRioHealthTable.getEntry("FPGAUptimeSeconds").setDouble(uptimeSeconds);
        roboRioHealthTable.getEntry("RebootedRecently").setBoolean(rebootedRecently);
        roboRioHealthTable.getEntry("RecentBootWindowSeconds").setDouble(kRecentBootWindowSeconds);
        roboRioHealthTable.getEntry("BrownedOut").setBoolean(brownedOutNow);
        roboRioHealthTable.getEntry("BatteryVoltage").setDouble(RobotController.getBatteryVoltage());
        roboRioHealthTable.getEntry("InputVoltage").setDouble(RobotController.getInputVoltage());
        roboRioHealthTable.getEntry("InputCurrent").setDouble(RobotController.getInputCurrent());
        roboRioHealthTable.getEntry("BrownoutVoltage").setDouble(RobotController.getBrownoutVoltage());
        roboRioHealthTable.getEntry("SystemActive").setBoolean(RobotController.isSysActive());
        roboRioHealthTable.getEntry("CommsDisableCount").setDouble(RobotController.getCommsDisableCount());
        roboRioHealthTable.getEntry("Enabled3V3").setBoolean(RobotController.getEnabled3V3());
        roboRioHealthTable.getEntry("Enabled5V").setBoolean(RobotController.getEnabled5V());
        roboRioHealthTable.getEntry("Enabled6V").setBoolean(RobotController.getEnabled6V());
        roboRioHealthTable.getEntry("FaultCount3V3").setDouble(RobotController.getFaultCount3V3());
        roboRioHealthTable.getEntry("FaultCount5V").setDouble(RobotController.getFaultCount5V());
        roboRioHealthTable.getEntry("FaultCount6V").setDouble(RobotController.getFaultCount6V());
        roboRioHealthTable.getEntry("Voltage3V3").setDouble(RobotController.getVoltage3V3());
        roboRioHealthTable.getEntry("Voltage5V").setDouble(RobotController.getVoltage5V());
        roboRioHealthTable.getEntry("Voltage6V").setDouble(RobotController.getVoltage6V());
        roboRioHealthTable.getEntry("Current3V3").setDouble(RobotController.getCurrent3V3());
        roboRioHealthTable.getEntry("Current5V").setDouble(RobotController.getCurrent5V());
        roboRioHealthTable.getEntry("Current6V").setDouble(RobotController.getCurrent6V());
        roboRioHealthTable.getEntry("CPUTempC").setDouble(RobotController.getCPUTemp());
        roboRioHealthTable.getEntry("Healthy").setBoolean(systemHealthy && !rebootedRecently);

        String status;
        if (brownedOutNow) {
            status = "Brownout active";
        } else if (rebootedRecently) {
            status = "RoboRIO booted within last " + kRecentBootWindowSeconds + " seconds";
        } else if (!RobotController.getEnabled3V3()) {
            status = "3.3V rail disabled";
        } else if (!RobotController.getEnabled5V()) {
            status = "5V rail disabled";
        } else if (!RobotController.getEnabled6V()) {
            status = "6V rail disabled";
        } else {
            status = "RoboRIO healthy";
        }

        roboRioHealthTable.getEntry("Status").setString(status);
    }


}
