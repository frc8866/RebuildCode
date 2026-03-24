package frc.robot.subsystems.Intake;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.sim.TalonFXSimState;
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

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Kilograms;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Pounds;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Intake extends SubsystemBase{
    private TalonFX intakeMotor = new TalonFX(11);
    private TalonFX intakeWheels = new TalonFX(16);
        Distance radius = Inches.of(1.5);

        double moi = Pounds.of(8.0).in(Kilograms) * Math.pow(radius.in(Meters), 2);




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
        public double intakeWheelsPosition = 0.0;
        public boolean isAtIntakePosition = false;
        public boolean isAtHomePosition = false;
        public String currentState = "IDLE";
        public String wantedState = "IDLE";
        public double intakeMotorTemp = 0;
        public double intakeWheelTemp = 0;
        

    }
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();

    private static final double kSimDtSeconds = 0.02;
    private static final double kIntakePositionRotations = 20;
    private static final double kHomePositionRotations = 0.0;
    private static final double kIntakeVoltage = 9;
    private static final double kHomingVoltage = -3;
    private static final double kPositionToleranceRotations = 1.0;
    private static final double kIntakeP = 0.5;
    private static final double kIntakeI = 0.0;
    private static final double kIntakeD = 0.0;
    private static final double kMaxIntakePidVoltage = 8.0;
    private static final double kShootPositionRotations = 0;
    private static final double kMaxShootPidVoltage = 1.5;
    private static final double kIntakeGearRatio = 10.0;
    private static final double kIntakeDrumRadiusMeters = 0.02;
    private static final double kIntakeMassKg = 2.0;
    private static final double kIntakeMetersPerRotation =
        (2.0 * Math.PI * kIntakeDrumRadiusMeters) / kIntakeGearRatio;
    private static final double kIntakeMaxMeters = kIntakePositionRotations * kIntakeMetersPerRotation;
    private static final double kRadiansPerRotation = 2.0 * Math.PI;

    private TalonFXSimState intakeMotorSim;
    private TalonFXSimState intakeWheelsSim;
    private ElevatorSim intakeElevatorSim;
    private DCMotorSim intakeWheelsMotorSim;
    private final PIDController intakePid =
        new PIDController(kIntakeP, kIntakeI, kIntakeD);

    public IntakeInputsAutoLogged inputs = new IntakeInputsAutoLogged();    
    public enum WantedState {
        IDLE,
        INTAKE,
        HOME,
        SHOOT,
        REVERSE
    }
    public enum CurrentState {
        IDLE,
        INTAKE,
        HOME,
        SHOOT,
        REVERSE
    }

    public WantedState wantedState = WantedState.IDLE;
    public CurrentState currentState = CurrentState.IDLE;
    private CurrentState lastState = CurrentState.IDLE;

    public Intake() {
        



        // Configure intake motors to coast (neutral) mode
        talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeMotor.getConfigurator().apply(talonFXConfigs);
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
        
    }

    private static double rpsToRadPerSec(double rps) {
        return rps * kRadiansPerRotation;
    }
    @Override
    public void periodic() {
        updateInputs();
        Logger.processInputs("Intake", inputs);
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
        inputs.intakeWheelsPosition = intakeWheels.getPosition().getValueAsDouble();
    double currentPos = intakeMotor.getPosition().getValueAsDouble();
    inputs.isAtIntakePosition = Math.abs(currentPos - kIntakePositionRotations) < 1.0;
    inputs.isAtHomePosition = Math.abs(currentPos - kHomePositionRotations) < 1.0;
        inputs.currentState = currentState.toString();  
        inputs.wantedState = wantedState.toString();
        inputs.intakeMotorTemp = intakeMotor.getDeviceTemp().getValueAsDouble();
        inputs.intakeWheelTemp = intakeWheels.getDeviceTemp().getValueAsDouble(); 


    }
    private void handleStates() {
        switch (wantedState) {
            case IDLE:
                currentState = CurrentState.IDLE;
                break;
            case INTAKE:
                currentState = CurrentState.INTAKE;
                break;
            case HOME:
                currentState = CurrentState.HOME;
                break;
            case SHOOT:
                currentState = CurrentState.SHOOT;
                break;
            case REVERSE:
                currentState = CurrentState.REVERSE;
                break;
        }

    }

    private void applyStates() {
        switch (currentState) {
            case IDLE:
                intakeMotor.setVoltage(0);
                intakeWheels.setVoltage(0);
                intakePid.reset();

                
                break;
            case INTAKE:
                double pos = intakeMotor.getPosition().getValueAsDouble();
                if (Math.abs(pos - kIntakePositionRotations) <= kPositionToleranceRotations) {
                    intakeMotor.setVoltage(0);
                    intakePid.reset();
                } else {
                    double output = intakePid.calculate(pos, kIntakePositionRotations);
                    
                    output = Math.max(-kMaxIntakePidVoltage, Math.min(kMaxIntakePidVoltage, output));
                    intakeMotor.setVoltage(output);
                }
                intakeWheels.setVoltage(-kIntakeVoltage);


                break;
            case HOME:
                double p = intakeMotor.getPosition().getValueAsDouble();
                double homeOutput = intakePid.calculate(p, kHomePositionRotations);
                homeOutput = Math.max(-kMaxIntakePidVoltage, Math.min(kMaxIntakePidVoltage, homeOutput));
                intakeMotor.setVoltage(homeOutput);
                intakeWheels.setVoltage(0);

                break;
            case SHOOT:
                double shootPos = intakeMotor.getPosition().getValueAsDouble();
                if (Math.abs(shootPos - kShootPositionRotations) <= kPositionToleranceRotations) {
                    intakeMotor.setVoltage(0);
                    intakePid.reset();
                } else {
                    if (currentState != lastState) {
                        intakePid.reset();
                    }
                    double shootOutput = intakePid.calculate(shootPos, kShootPositionRotations);
                    shootOutput = Math.max(-kMaxShootPidVoltage, Math.min(kMaxShootPidVoltage, shootOutput));
                    intakeMotor.setVoltage(shootOutput);
                }
                intakeWheels.setVoltage(-4);
                break;
            case REVERSE:
                double reversePos = intakeMotor.getPosition().getValueAsDouble();
                if (Math.abs(reversePos - kIntakePositionRotations) <= kPositionToleranceRotations) {
                    intakeMotor.setVoltage(0);
                    intakePid.reset();
                } else {
                    double reverseOutput = intakePid.calculate(reversePos, kIntakePositionRotations);
                    reverseOutput = Math.max(-kMaxIntakePidVoltage, Math.min(kMaxIntakePidVoltage, reverseOutput));
                    intakeMotor.setVoltage(reverseOutput);
                }
                intakeWheels.setVoltage(kIntakeVoltage);
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


    /** Stop all intake activity and hold position. */
    public void stop() {
        wantedState = WantedState.IDLE;
    }

    
}
