package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Shooter;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Intake extends SubsystemBase{
    private TalonFX intakeMotor = new TalonFX(14);
    private TalonFX intakeMotorFollower = new TalonFX(15);
    private TalonFX intakeWheels = new TalonFX(19);
    private TalonFX intakeWheelsFollower = new TalonFX(16);

        Distance radius = Inches.of(1.5);

        double moi = Pounds.of(8.0).in(Kilograms) * Math.pow(radius.in(Meters), 2);

    public Shooter shooter;


    @AutoLog
    public static class IntakeInputs {
        public boolean intakeMotorConnected = false;
        public boolean intakeWheelsConnected = false;
        public double intakeMotorVoltage = 0.0;
        public double intakeWheelsVoltage = 0.0;
        public double intakeMotorCurrent = 0.0;
        public double intakeWheelsCurrent = 0.0;
        public double intakeMotorSpeedVelocity = 0.0;
        public double intakeWheelsSpeedVelocity = 0.0;
        public double intakeMotorPosition = 0.0;
        public double intakeMotorFollowerPosition = 0.0;
        public double intakeWheelsPosition = 0.0;
        public boolean isAtIntakePosition = false;
        public boolean isAtHomePosition = false;
        public String currentState = "IDLE";
        public String wantedState = "IDLE";
        public double intakeMotorTemp = 0;
        public double intakeWheelTemp = 0;
        public double followerEncoderDiff = 0.0;


    }
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    private static final double kSimDtSeconds = 0.02;
    private static final double kIntakePositionRotations = 0.0; // all the way out (encoder zero)
    private static final double kIntakeHALFPositionRotations = kIntakePositionRotations/2;
    private static final double kHomePositionRotations = 10.5; // all the way in
    private static final double kIntakeVoltage = 6;
    private static final double kHomingVoltage = -3;
    private static final double kPositionToleranceRotations = 1.0;
    private static final double kIntakeP = 0.8;
    private static final double kIntakeI = 0.0;
    private static final double kIntakeD = 0.0;
    private static final double kMaxIntakePidVoltage = 4;
    private static final double kShootPositionRotations = 12;
    //12.24697265625
     // all the way in (same as HOME)
    private static final double kMaxShootPidVoltage = 1;

    private static final double kIntakeGearRatio = 10.0;
    private static final double kIntakeDrumRadiusMeters = 0.02;
    private static final double kIntakeMassKg = 2.0;
    private static final double kIntakeMetersPerRotation =
        (2.0 * Math.PI * kIntakeDrumRadiusMeters) / kIntakeGearRatio;
    private static final double kIntakeMaxMeters = kIntakePositionRotations * kIntakeMetersPerRotation;
    private static final double kRadiansPerRotation = 2.0 * Math.PI;
    private Timer shootTimer = new Timer();


    private TalonFXSimState intakeMotorSim;
    private TalonFXSimState intakeWheelsSim;
    private ElevatorSim intakeElevatorSim;
    private DCMotorSim intakeWheelsMotorSim;

    // Main motor PID (uses intakeMotor encoder)
    private final PIDController intakePid =
        new PIDController(kIntakeP, kIntakeI, kIntakeD); 
          private final PIDController intakePid1 =
        new PIDController(0.09, kIntakeI, kIntakeD);

    // Follower motor PID (uses intakeMotorFollower encoder independently)
    // Since the follower is physically opposed, its encoder reads in the opposite
    // direction — so we negate the target when commanding it.
    private final PIDController intakeFollowerPid =
        new PIDController(kIntakeP, kIntakeI, kIntakeD);
                 private final PIDController intakePidfollower1 =
        new PIDController(0.09, kIntakeI, kIntakeD);

    private Follower wheels_follower = new Follower(intakeWheels.getDeviceID(), MotorAlignmentValue.Opposed);

    public IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();
    public enum WantedState {
        IDLE,
        INTAKE,
        HOME,
        SHOOT,
        REVERSE,
        INTAKE_HOLD
    }
    public enum CurrentState {
        IDLE,
        INTAKE,
        HOME,
        SHOOT,
        REVERSE,
        INTAKE_HOLD
    }

    public WantedState wantedState = WantedState.IDLE;
    public CurrentState currentState = CurrentState.IDLE;
    private CurrentState lastState = CurrentState.IDLE;

    public Intake() {

        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        talonFXConfigs.CurrentLimits.StatorCurrentLimit = 60;
        talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimit = 30;
        talonFXConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;

        intakeMotor.getConfigurator().apply(talonFXConfigs);
        intakeMotorFollower.getConfigurator().apply(talonFXConfigs);

        intakeWheels.getConfigurator().apply(talonFXConfigs);

        if (RobotBase.isSimulation()) {
            intakeMotorSim = intakeMotor.getSimState();
            intakeWheelsSim = intakeWheels.getSimState();
            intakeElevatorSim =
                new ElevatorSim(
                    DCMotor.getKrakenX60(1),
                    kIntakeGearRatio,
                    kIntakeMassKg,
                    kIntakeDrumRadiusMeters,
                    0.0,
                    kIntakeMaxMeters,
                    false,
                    0.0);

     LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(1), moi, 1.5);
            intakeWheelsMotorSim = new DCMotorSim(linearSystem, DCMotor.getKrakenX60(1));
        }

        intakePid.setTolerance(kPositionToleranceRotations);
        intakeFollowerPid.setTolerance(kPositionToleranceRotations);

        // Wheels follower still mirrors the wheels leader
        intakeWheelsFollower.setControl(wheels_follower);
    }

    private static double rpsToRadPerSec(double rps) {
        return rps * kRadiansPerRotation;
    }

    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Intake", inputs);
        Logger.recordOutput("Intake111", intakeMotor.getStatorCurrent().getValueAsDouble());
        handleStates();
        applyStates();
        lastState = currentState;
    }

    @Override
    public void simulationPeriodic() {
        if (intakeElevatorSim == null) {
            return;
        }

        double batteryVoltage = RobotController.getBatteryVoltage();

        intakeElevatorSim.setInputVoltage(intakeMotor.getMotorVoltage().getValueAsDouble());
        intakeElevatorSim.update(kSimDtSeconds);

        double positionMeters = intakeElevatorSim.getPositionMeters();
        double velocityMetersPerSec = intakeElevatorSim.getVelocityMetersPerSecond();
        double motorRotations = positionMeters / kIntakeMetersPerRotation;
        double motorRotationsPerSec = velocityMetersPerSec / kIntakeMetersPerRotation;

        intakeMotorSim.setSupplyVoltage(Volts.of(batteryVoltage));
        intakeMotorSim.setRawRotorPosition(Rotations.of(motorRotations));
        intakeMotorSim.setRotorVelocity(RotationsPerSecond.of(motorRotationsPerSec));

        intakeWheelsMotorSim.setInputVoltage(intakeWheels.getMotorVoltage().getValueAsDouble());
        intakeWheelsMotorSim.update(kSimDtSeconds);
        intakeWheelsSim.setSupplyVoltage(Volts.of(batteryVoltage));
        intakeWheelsSim.setRawRotorPosition(Rotations.of(intakeWheelsMotorSim.getAngularPositionRotations()));
        intakeWheelsSim.setRotorVelocity(
            RotationsPerSecond.of(intakeWheelsMotorSim.getAngularVelocityRPM() / 60.0));
    }

    private void updateInputs() {
        inputs.intakeMotorConnected = intakeMotor.isConnected();
        inputs.intakeWheelsConnected = intakeWheels.isConnected();
        inputs.intakeMotorVoltage = intakeMotor.getMotorVoltage().getValueAsDouble();
        inputs.intakeWheelsVoltage = intakeWheels.getMotorVoltage().getValueAsDouble();
        inputs.intakeMotorCurrent = intakeMotor.getStatorCurrent().getValueAsDouble();
        inputs.intakeWheelsCurrent = intakeWheels.getStatorCurrent().getValueAsDouble();
        inputs.intakeMotorSpeedVelocity =
            rpsToRadPerSec(intakeMotor.getVelocity().getValueAsDouble());
        inputs.intakeWheelsSpeedVelocity =
            rpsToRadPerSec(intakeWheels.getVelocity().getValueAsDouble());
        inputs.intakeMotorPosition = intakeMotor.getPosition().getValueAsDouble();
        inputs.intakeMotorFollowerPosition = intakeMotorFollower.getPosition().getValueAsDouble();
        inputs.intakeWheelsPosition = intakeWheels.getPosition().getValueAsDouble();
        double currentPos = intakeMotor.getPosition().getValueAsDouble();
        inputs.isAtIntakePosition = Math.abs(currentPos - kIntakePositionRotations) < 1.0;
        inputs.isAtHomePosition = Math.abs(currentPos - kHomePositionRotations) < 1.0;
        inputs.currentState = currentState.toString();
        inputs.wantedState = wantedState.toString();
        inputs.intakeMotorTemp = intakeMotor.getDeviceTemp().getValueAsDouble();
        inputs.intakeWheelTemp = intakeWheels.getDeviceTemp().getValueAsDouble();
        // followerEncoderDiff: main pos vs negated follower pos (both should track together)
        inputs.followerEncoderDiff = currentPos - (-intakeMotorFollower.getPosition().getValueAsDouble());
    }

    private void handleStates() {
        switch (wantedState) {
            case HOME:
                currentState = CurrentState.HOME;
                break;
            case IDLE:
                currentState = CurrentState.IDLE;
                break;
            case INTAKE:
                currentState = CurrentState.INTAKE;
                break;
            case SHOOT:
                currentState = CurrentState.SHOOT;
                break;
            case REVERSE:
                currentState = CurrentState.REVERSE;
                break;
            case INTAKE_HOLD:
                currentState = CurrentState.INTAKE_HOLD;
                break;
        }
    }

    /**
     * Apply voltage to both arm motors independently toward the same target.
     * intakeMotorFollower is physically opposed so its target is negated.
     */
    private void setArmVoltage(double mainOutput, double clampMax) {
        double clamped = Math.max(-clampMax, Math.min(clampMax, mainOutput));
        intakeMotor.setVoltage(clamped);

        double followerPos = intakeMotorFollower.getPosition().getValueAsDouble();
        double followerTarget = -intakePid.getSetpoint(); // negate target for opposed motor
        double followerOutput = intakeFollowerPid.calculate(followerPos, followerTarget);
        followerOutput = Math.max(-clampMax, Math.min(clampMax, followerOutput));
        intakeMotorFollower.setVoltage(followerOutput);
    }
        private void setArmVoltage1(double mainOutput, double clampMax) {
        double clamped = Math.max(-clampMax, Math.min(clampMax, mainOutput));
        intakeMotor.setVoltage(clamped);

        double followerPos = intakeMotorFollower.getPosition().getValueAsDouble();
        double followerTarget = -intakePid1.getSetpoint(); // negate target for opposed motor
        double followerOutput = intakePidfollower1.calculate(followerPos, followerTarget);
        followerOutput = Math.max(-clampMax, Math.min(clampMax, followerOutput));
        intakeMotorFollower.setVoltage(followerOutput);
    }


    private void stopArmMotors() {
        intakeMotor.setVoltage(0);
        intakeMotorFollower.setVoltage(0);
        intakePid.reset();
        intakeFollowerPid.reset();
    }

    private void applyStates() {
        switch (currentState) {
            case IDLE:
                stopArmMotors();
                intakeWheels.setVoltage(0);
                break;

            case INTAKE:
                double pos = intakeMotor.getPosition().getValueAsDouble();
                double followerPos = intakeMotorFollower.getPosition().getValueAsDouble();
                boolean bothAtIntake = Math.abs(pos - kIntakePositionRotations) <= kPositionToleranceRotations
                        && Math.abs(followerPos - (-kIntakePositionRotations)) <= kPositionToleranceRotations;
                if (bothAtIntake) {
                    stopArmMotors();
                } else {
                    double output = intakePid.calculate(pos, kIntakePositionRotations);
                    setArmVoltage(output, kMaxIntakePidVoltage);
                }
                intakeWheels.setVoltage(kIntakeVoltage); //kIntakeVoltage
                shooter.whaletail(4);

                break;

            case HOME:
                double p = intakeMotor.getPosition().getValueAsDouble();
                double homeOutput = intakePid.calculate(p, kHomePositionRotations);
                setArmVoltage(homeOutput, kMaxIntakePidVoltage);
                intakeWheels.setVoltage(0);
                break;

            case SHOOT:
                double shootPos2 = intakeMotor.getPosition().getValueAsDouble();
                if (Math.abs(shootPos2 - kShootPositionRotations) <= kPositionToleranceRotations) {
                    stopArmMotors();
                } else {
                    if (currentState != lastState) {
                        intakePid.reset();
                        intakeFollowerPid.reset();
                    }
                    double shootOutput = intakePid1.calculate(shootPos2, kShootPositionRotations);
                    setArmVoltage1(shootOutput, kMaxShootPidVoltage); // intentionally 0 voltage as in original
                }
                intakeWheels.setVoltage(kIntakeVoltage);//kIntakeVoltage
                break;

            case REVERSE:
                double reversePos = intakeMotor.getPosition().getValueAsDouble();
                if (Math.abs(reversePos - kIntakePositionRotations) <= kPositionToleranceRotations) {
                    stopArmMotors();
                } else {
                    double reverseOutput = intakePid.calculate(reversePos, kIntakePositionRotations);
                    setArmVoltage(reverseOutput, kMaxIntakePidVoltage);
                }
                intakeWheels.setVoltage(-kIntakeVoltage); //-kIntakeVoltage
                shooter.whaletail(4);
                break;

            case INTAKE_HOLD:
                stopArmMotors();
                intakeWheels.setVoltage(kIntakeVoltage); //kIntakeVoltage
                break;
        }
    }

    // Convenience control methods
    /** Start the intake motion (move to intake position and run wheels). */
    public void startIntake() {
        wantedState = WantedState.INTAKE;
    }

    /** Move intake to home (zero) position and stop wheels. */
    public void goHome() {
        wantedState = WantedState.HOME;
    }

    public void shoot() {
        wantedState = WantedState.SHOOT;
    }

    public void reverse() {
        wantedState = WantedState.REVERSE;
    }

    /** Hold arm at current position and keep wheels running. */
    public void intakeHold() {
        wantedState = WantedState.INTAKE_HOLD;
    }

    /** Stop all intake activity and hold position. */
    public void stop() {
        wantedState = WantedState.IDLE;
    }


}
