package frc.robot.subsystems;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.sim.CANcoderSimState;
import com.ctre.phoenix6.sim.TalonFXSimState;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.function.Supplier;

public class Turret extends SubsystemBase {

    // ---------------------------------------------------------------
    // CAN IDs — update these to match your robot wiring
    // ---------------------------------------------------------------
    private static final int kLeaderMotorId   = 30;
    private static final int kFollowerMotorId = 31;
    private static final int kCANcoderId      = 26;

    // ---------------------------------------------------------------
    // Physical / mechanical constants
    // ---------------------------------------------------------------
    /** Motor rotations per one full turret rotation. */
    private static final double kGearRatio = 50.0;

    /** Moment of inertia of the turret plate (kg·m²) — tune in sim. */
    private static final double kTurretMOI = 0.05;

    // ---------------------------------------------------------------
    // Control gains — tune on real hardware / in sim
    // ---------------------------------------------------------------
    private static final double kP = 80.0;
    private static final double kI = 0.0;
    private static final double kD = 0.5;
    private static final double kS = 0.25;
    private static final double kV = 0.12;
    private static final double kA = 0.0;

    /** Motion Magic cruise velocity in turret rotations/sec. */
    private static final double kCruiseVelRPS = 100;

    /** Motion Magic acceleration in turret rotations/sec². */
    private static final double kAccelRPS2 = 100;

    private static final double kToleranceDegrees = 1.0;
    private static final double kSimDtSeconds     = 0.02;

    // ---------------------------------------------------------------
    // Hardware
    // ---------------------------------------------------------------
    private final TalonFX            leaderMotor      = new TalonFX(kLeaderMotorId);
    private final TalonFX            followerMotor    = new TalonFX(kFollowerMotorId);
    private final CANcoder           turretEncoder    = new CANcoder(kCANcoderId);
    private final MotionMagicVoltage positionRequest  = new MotionMagicVoltage(0);
    private final Follower           followerRequest  = new Follower(kLeaderMotorId, MotorAlignmentValue.Opposed);

    // ---------------------------------------------------------------
    // AdvantageKit logging
    // ---------------------------------------------------------------
    @AutoLog
    public static class TurretInputs {
        public boolean leaderConnected   = false;
        public boolean followerConnected = false;
        public boolean encoderConnected  = false;
        public double  currentAngleDeg   = 0.0;
        public double  targetAngleDeg    = 0.0;
        public double  leaderVoltage     = 0.0;
        public double  followerVoltage   = 0.0;
        public double  leaderCurrent     = 0.0;
        public double  followerCurrent   = 0.0;
        public double  leaderTemp        = 0.0;
        public double  followerTemp      = 0.0;
        public boolean atTarget          = false;
        public String  currentState      = "IDLE";
        public String  wantedState       = "IDLE";
    }

    // ---------------------------------------------------------------
    // State machine
    // ---------------------------------------------------------------
    public enum WantedState  { IDLE, MOVING_TO_ANGLE }
    public enum CurrentState { IDLE, MOVING_TO_ANGLE }

    public  WantedState  wantedState  = WantedState.IDLE;
    public  CurrentState currentState = CurrentState.IDLE;
    private double       targetAngleDeg = 0.0;
    /** Field-relative heading (degrees) the turret locks to while in IDLE. */
    private double       idleFieldHeadingDeg = 0.0;

    private final TurretInputsAutoLogged inputs = new TurretInputsAutoLogged();

    // ---------------------------------------------------------------
    // Simulation
    // ---------------------------------------------------------------
    private TalonFXSimState  leaderSim;
    private CANcoderSimState encoderSim;
    private DCMotorSim       turretMotorSim;

    private final Supplier<Pose2d> poseSupplier;

    // ---------------------------------------------------------------
    // Constructor
    // ---------------------------------------------------------------
    public Turret(Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;

        // CANcoder — set magnet offset via Phoenix Tuner if needed
        turretEncoder.getConfigurator().apply(new CANcoderConfiguration());

        // Leader motor
        TalonFXConfiguration leaderConfig = new TalonFXConfiguration();
        leaderConfig.Slot0 = new Slot0Configs()
            .withKP(kP).withKI(kI).withKD(kD)
            .withKS(kS).withKV(kV).withKA(kA);
        leaderConfig.MotionMagic = new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(kCruiseVelRPS)
            .withMotionMagicAcceleration(kAccelRPS2)
            .withMotionMagicJerk(0);
        // FusedCANcoder requires Phoenix Pro; swap to RemoteCANcoder if not on Pro
        leaderConfig.Feedback = new FeedbackConfigs()
            .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder)
            .withFeedbackRemoteSensorID(kCANcoderId)
            .withSensorToMechanismRatio(1.0)   // CANcoder mounted on turret output shaft
            .withRotorToSensorRatio(kGearRatio);
        leaderConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        leaderMotor.getConfigurator().apply(leaderConfig);

        // Follower motor — mirrors leader, same direction
        TalonFXConfiguration followerConfig = new TalonFXConfiguration();
        followerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        followerMotor.getConfigurator().apply(followerConfig);
        followerMotor.setControl(followerRequest);

        // Simulation setup
        if (RobotBase.isSimulation()) {
            leaderSim  = leaderMotor.getSimState();
            encoderSim = turretEncoder.getSimState();

            LinearSystem<N2, N1, N2> plant = LinearSystemId.createDCMotorSystem(
                DCMotor.getKrakenX60(2), kTurretMOI, kGearRatio);
            turretMotorSim = new DCMotorSim(plant, DCMotor.getKrakenX60(2));
        }
    }

    // ---------------------------------------------------------------
    // Periodic loops
    // ---------------------------------------------------------------
    @Override
    public void periodic() {
        updateInputs();
        handleStates();
        applyStates();
        Logger.processInputs("Turret", inputs);
        Logger.recordOutput("Turret/AngleDeg",  inputs.currentAngleDeg);
        Logger.recordOutput("Turret/TargetDeg", targetAngleDeg);
    }

    @Override
    public void simulationPeriodic() {
        if (turretMotorSim == null) return;

        double batteryVoltage = RobotController.getBatteryVoltage();

        // Step the physics sim
        turretMotorSim.setInputVoltage(leaderMotor.getMotorVoltage().getValueAsDouble());
        turretMotorSim.update(kSimDtSeconds);

        // turretRotations is in terms of the turret output shaft
        double turretRotations   = turretMotorSim.getAngularPositionRotations();
        double turretRotationsPS = turretMotorSim.getAngularVelocityRPM() / 60.0;

        // Feed back into TalonFX (rotor side) and CANcoder (output side)
        leaderSim.setSupplyVoltage(Volts.of(batteryVoltage));
        leaderSim.setRawRotorPosition(Rotations.of(turretRotations * kGearRatio));
        leaderSim.setRotorVelocity(RotationsPerSecond.of(turretRotationsPS * kGearRatio));

        encoderSim.setSupplyVoltage(Volts.of(batteryVoltage));
        encoderSim.setRawPosition(Rotations.of(turretRotations));
        encoderSim.setVelocity(RotationsPerSecond.of(turretRotationsPS));
    }

    // ---------------------------------------------------------------
    // Private helpers
    // ---------------------------------------------------------------
    private void updateInputs() {
        inputs.leaderConnected   = leaderMotor.isConnected();
        inputs.followerConnected = followerMotor.isConnected();
        inputs.encoderConnected  = turretEncoder.isConnected();
        // CANcoder reports in rotations (0–1 per revolution); convert to degrees
        inputs.currentAngleDeg   = turretEncoder.getPosition().getValueAsDouble() * 360.0;
        inputs.targetAngleDeg    = targetAngleDeg;
        inputs.leaderVoltage     = leaderMotor.getMotorVoltage().getValueAsDouble();
        inputs.followerVoltage   = followerMotor.getMotorVoltage().getValueAsDouble();
        inputs.leaderCurrent     = leaderMotor.getStatorCurrent().getValueAsDouble();
        inputs.followerCurrent   = followerMotor.getStatorCurrent().getValueAsDouble();
        inputs.leaderTemp        = leaderMotor.getDeviceTemp().getValueAsDouble();
        inputs.followerTemp      = followerMotor.getDeviceTemp().getValueAsDouble();
        inputs.atTarget          = Math.abs(inputs.currentAngleDeg - targetAngleDeg) < kToleranceDegrees;
        inputs.currentState      = currentState.toString();
        inputs.wantedState       = wantedState.toString();
    }

    private void handleStates() {
        switch (wantedState) {
            case IDLE:
                if (currentState != CurrentState.IDLE) {
                    // Lock the current field-relative heading on transition into IDLE
                    double robotHeadingDeg = poseSupplier.get().getRotation().getDegrees();
                    idleFieldHeadingDeg = inputs.currentAngleDeg + robotHeadingDeg;
                }
                currentState = CurrentState.IDLE;
                break;
            case MOVING_TO_ANGLE: currentState = CurrentState.MOVING_TO_ANGLE; break;
        }
    }

    private void applyStates() {
        switch (currentState) {
            case IDLE:
                // Compensate for robot rotation to hold the locked field-relative heading
                double robotHeadingDeg = poseSupplier.get().getRotation().getDegrees();
                double compensatedAngle = wrapDegrees(idleFieldHeadingDeg - robotHeadingDeg);
                leaderMotor.setControl(positionRequest.withPosition(compensatedAngle / 360.0));
                break;
            case MOVING_TO_ANGLE:
                // MotionMagic position target in turret rotations (CANcoder units)
                double targetRotations = targetAngleDeg / 360.0;
                leaderMotor.setControl(positionRequest.withPosition(targetRotations));
                break;
        }
    }

    // ---------------------------------------------------------------
    // Angle wrapping
    //   370  → 10
    //   500  → 140  (500 mod 360 = 140)
    //  -10   → 350
    // ---------------------------------------------------------------
    public static double wrapDegrees(double degrees) {
        return ((degrees % 360.0) + 360.0) % 360.0;
    }

    // ---------------------------------------------------------------
    // Public API
    // ---------------------------------------------------------------

    /**
     * Command the turret to a target angle. Any value outside [0, 360) is
     * automatically wrapped — e.g. 370 → 10°, 500 → 140°, -10 → 350°.
     */
    public void moveToAngle(double angleDegrees) {
        targetAngleDeg = wrapDegrees(angleDegrees);
        wantedState    = WantedState.MOVING_TO_ANGLE;
    }

    /** Stop the turret (zero voltage, hold last position via brake mode). */
    public void setIdle() {
        wantedState = WantedState.IDLE;
    }

    public boolean isAtTarget() {
        return inputs.atTarget;
    }

    public double getCurrentAngleDegrees() {
        return inputs.currentAngleDeg;
    }

    public double getTargetAngleDegrees() {
        return targetAngleDeg;
    }
}
