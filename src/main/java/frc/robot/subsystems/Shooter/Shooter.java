package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Velocity;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.TalonFXSimState;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

public class Shooter extends SubsystemBase {
      private final VoltageOut shootVoltageRequest = new VoltageOut(0);



  private final DCMotor dcMotor = DCMotor.getKrakenX60(2);

  private final DCMotor dcMotorSpinner = DCMotor.getKrakenX60(1);
  private final DCMotor dcMotorWheel = DCMotor.getKrakenX60(1);
  

  private final TalonFX shooterLeaderMotor = new TalonFX(1);
  private final TalonFX shooterFollower = new TalonFX(2);
  private final TalonFX spindexerSpinnerMotor = new TalonFX(18);
  private final TalonFX spindexerWheelMotor = new TalonFX(20);

  private static final double kSimDtSeconds = 0.02;
  private static final double kShooterSimMoi = 0.004418;
  private static final double kSpindexerSimMoi = 0.005;
  private static final double kSimGearRatio = 1;
  private static final double kSpindexerPulseSeconds = 0.25;

  private TalonFXSimState shooterLeaderSim;
  private TalonFXSimState shooterFollowerSim;
  private TalonFXSimState spindexerSpinnerSim;
  private TalonFXSimState spindexerWheelSim;

  private DCMotorSim flyWheelSIM;
  private DCMotorSim spindexerSpinnerMotorSim;
  private DCMotorSim spindexerWheelMotorSim;
  public boolean spindexer; 


 
  public double targetShooterVelocity = 0.0;
  public double targetSpindexerWheelVelocity = 0.0;
  
      @AutoLog
  public static class shooterInputs{
    public double shooterVelocity = 0.0;
    public double spindexerSpinnerVelocity = 0.0;
    public double spindexerWheelVelocity = 0.0;
    public boolean shooterAtVelocity = false;
    public boolean spindexerWheelAtVelocity = false;
    public double targetVelocity = 0.0;
    public double targetRPM = 0.0;
    public double shooterAppliedVoltage = 0.0;
    public double spindexerSpinnerAppliedVoltage = 0.0;
    public double spindexerWheelAppliedVoltage = 0.0;
    public double shooterCurrent = 0.0;
    public String currentState = "OFF";
    public String wantedState = "OFF";
    public boolean shooterMotorConnected = false;
    public boolean shooterMotorFollower = false;
    public boolean spindexerSpinnerMotorConnected = false;
    public boolean spindexerWheelMotorConnected = false;
    public double shooterVelocityMPS = 0;
    public double shooterRPM = 0;

 
  }
  private shooterInputsAutoLogged inputs = new shooterInputsAutoLogged();




  private final VelocityVoltage shooterVoltage = new VelocityVoltage(0);

  public double testRpm = 0.0;

  public enum WantedState {
    OFF,
    PRE_SHOOT,
    SHOOT,
    SHOOTAREA,
    DASHBOARD,
    TEST

  }

  public enum CurrentState {
    OFF,
    PRE_SHOOT,
    SHOOT,
    SHOOTAREA,
    DASHBOARD,
    TEST


  }

  private WantedState wantedState = WantedState.OFF;
  private CurrentState currentState = CurrentState.OFF;

  private double targetRPS = 0.0;
  private double spindexerPulseUntilSeconds = 0.0;
  private boolean spindexerManualEnabled = false;
  private boolean spindexerManualReverse = false;
  private Follower follower = new Follower(shooterLeaderMotor.getDeviceID(), MotorAlignmentValue.Opposed);


  public Shooter() {

    
   TalonFXConfiguration config = new TalonFXConfiguration();
        config.Slot0.kV = 0.121934; // ~12V at 100 rps (6000 rpm) for Kraken X60
        config.Slot0.kP = 0.5;
        config.CurrentLimits.StatorCurrentLimit = 50;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 70;



  // Set motors to coast (neutral) mode for shooter leader and follower
  config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    shooterFollower.setControl(follower);

  shooterLeaderMotor.getConfigurator().apply(config);
  shooterFollower.getConfigurator().apply(config);

  // Preserve existing application for spindexer wheel as well

  if (RobotBase.isSimulation()) {
    shooterLeaderSim = shooterLeaderMotor.getSimState();
    shooterFollowerSim = shooterFollower.getSimState();
    spindexerSpinnerSim = spindexerSpinnerMotor.getSimState();
    spindexerWheelSim = spindexerWheelMotor.getSimState();


     LinearSystem<N2, N1, N2> linearSystem =
        LinearSystemId.createDCMotorSystem(
            dcMotor, kShooterSimMoi, kSimGearRatio); 

            LinearSystem<N2, N1, N2> linearSystemSpinner =
        LinearSystemId.createDCMotorSystem(
            dcMotor, kShooterSimMoi, kSimGearRatio); 

    flyWheelSIM = new DCMotorSim(linearSystem, dcMotor);
    spindexerSpinnerMotorSim = new DCMotorSim(linearSystemSpinner, dcMotorSpinner);
    spindexerWheelMotorSim = new DCMotorSim(linearSystem, dcMotorWheel);

        
  }
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Shooter/ShooterVelocity", shooterLeaderMotor.getVelocity().getValueAsDouble());
    updateInputs();
    Logger.processInputs("Shooter", inputs);

    Logger.recordOutput("Voltageout", shootVoltageRequest.getOutputMeasure());
    Logger.recordOutput("spin", spindexer);

    handleStates();
    applyStates();
  }

  @Override
  public void simulationPeriodic() {
    if (shooterLeaderSim == null) {
      return;
    }

    double batteryVoltage = RobotController.getBatteryVoltage();
    updateMotorSim(
        flyWheelSIM,
        shooterLeaderSim,
        shooterLeaderMotor.getMotorVoltage().getValueAsDouble(),
        batteryVoltage);
    updateMotorSim(
        spindexerSpinnerMotorSim,
        spindexerSpinnerSim,
        spindexerSpinnerMotor.getMotorVoltage().getValueAsDouble(),
        batteryVoltage);
    updateMotorSim(
        spindexerWheelMotorSim,
        spindexerWheelSim,
        spindexerWheelMotor.getMotorVoltage().getValueAsDouble(),
        batteryVoltage);
   
  }

  private void updateMotorSim(
      DCMotorSim motorSim, TalonFXSimState simState, double inputVoltage, double batteryVoltage) {
    motorSim.setInputVoltage(inputVoltage);
    motorSim.update(kSimDtSeconds);
    simState.setSupplyVoltage(Volts.of(batteryVoltage));
    simState.setRawRotorPosition(Radians.of(motorSim.getAngularPositionRad()));
    simState.setRotorVelocity(RadiansPerSecond.of(motorSim.getAngularVelocityRadPerSec()));
  }
  private void updateInputs(){
    inputs.shooterVelocity = shooterLeaderMotor.getVelocity().getValueAsDouble();
    inputs.spindexerSpinnerVelocity = spindexerSpinnerMotor.getVelocity().getValueAsDouble();
    inputs.spindexerWheelVelocity = spindexerWheelMotor.getVelocity().getValueAsDouble();
    inputs.shooterAtVelocity = Math.abs(shooterLeaderMotor.getVelocity().getValueAsDouble() - targetRPS) < 3;
    inputs.targetVelocity = targetRPS;
    inputs.shooterAppliedVoltage = shooterLeaderMotor.getMotorVoltage().getValueAsDouble();
    inputs.shooterCurrent = shooterLeaderMotor.getStatorCurrent().getValueAsDouble();
    inputs.currentState = currentState.toString();
    inputs.wantedState = wantedState.toString();
    inputs.shooterMotorConnected = shooterLeaderMotor.isConnected();
    inputs.shooterMotorFollower = shooterFollower.isConnected();
    inputs.spindexerSpinnerMotorConnected = spindexerSpinnerMotor.isConnected();
    inputs.spindexerWheelMotorConnected = spindexerWheelMotor.isConnected();
    inputs.spindexerWheelAppliedVoltage = spindexerWheelMotor.getMotorVoltage().getValueAsDouble();
    inputs.spindexerSpinnerAppliedVoltage = spindexerSpinnerMotor.getMotorVoltage().getValueAsDouble();
    inputs.shooterVelocityMPS = getLinearVelocity();
    inputs.targetRPM = targetRPS * 60;
    inputs.shooterRPM = shooterLeaderMotor.getVelocity().getValueAsDouble() * 60;
    

  }

  public double getangularVelocity(){
    return shooterLeaderMotor.getVelocity().getValueAsDouble() * 2 * Math.PI;
  }

  public double getLinearVelocity(){
    return getangularVelocity()* Units.inchesToMeters(2);
  }

  private void handleStates() {
    switch (wantedState) {
      case OFF:
        currentState = CurrentState.OFF;
        break;
      case PRE_SHOOT:
        currentState = CurrentState.PRE_SHOOT;
        break;
      case SHOOT:
        if(shooterLeaderMotor.getVelocity().getValueAsDouble() > targetRPS-4){
          currentState = CurrentState.SHOOT;
        }
        else if(shooterLeaderMotor.getVelocity().getValueAsDouble() < targetRPS- 4){
          currentState = CurrentState.PRE_SHOOT;
        }
        break;
      case SHOOTAREA:
      currentState = CurrentState.SHOOTAREA;  
        break;
      case DASHBOARD:
        currentState = CurrentState.DASHBOARD;
        break;
      case TEST:
        currentState = CurrentState.TEST;
        break;
    }
    
  }

  private void applyStates() {

    switch (currentState) {
      case OFF:
        shooterLeaderMotor.setVoltage(0);;
        spindexerSpinnerMotor.setVoltage(0);
        spindexerWheelMotor.setVoltage(0);;

        break;

      case PRE_SHOOT:
      shooterLeaderMotor.setControl(shooterVoltage.withVelocity(targetRPS));

      if(isAtVelocity() && spindexer){
         spindexerSpinnerMotor.setVoltage(7);
         
        spindexerWheelMotor.setVoltage(-12);


      }
      else{
                spindexerSpinnerMotor.setVoltage(0);
        spindexerWheelMotor.setVoltage(0);
      }

       
        break;
      case SHOOT:
          shooterLeaderMotor.setControl(shooterVoltage.withVelocity(targetRPS));

        spindexerSpinnerMotor.setVoltage(7);
        spindexerWheelMotor.setVoltage(-12);
        break;
      case SHOOTAREA:
        shooterLeaderMotor.setVoltage(2);;
        spindexerSpinnerMotor.setVoltage(0);
        spindexerWheelMotor.setVoltage(0);
        break;

      case DASHBOARD:
        shooterLeaderMotor.setControl(shooterVoltage.withVelocity(targetRPS));
        if (spindexerManualEnabled || spindexerManualReverse) {
          setSpindexerManualVoltages();
        } else {
          spindexerSpinnerMotor.setVoltage(0);
          spindexerWheelMotor.setVoltage(0);
        }
        break;

      case TEST:
      break;
        
    }

    if (Timer.getFPGATimestamp() < spindexerPulseUntilSeconds) {
     setSpindexerManualVoltages();
    }
    if (spindexerManualEnabled) {
      setSpindexerManualVoltages();
    }
  }


  public void whaletail(double voltage) {
    spindexerSpinnerMotor.setVoltage(-voltage);
    spindexerWheelMotor.setVoltage(10);
  }

  public void setOff() {
    wantedState = WantedState.OFF;
    targetRPS = 0.0;
    spindexerManualEnabled = false;
    spindexerManualReverse = false;
  }

  public double rps(){
    return shooterLeaderMotor.getVelocity().getValueAsDouble();
  }


  public void shoot(double rps){
    targetRPS = rps;
    wantedState = WantedState.SHOOT;
    spindexerManualEnabled = false;
    spindexerManualReverse = false;

  }

  public void preShoot(double rps,boolean spindexer){
    targetRPS = rps;
    wantedState = WantedState.PRE_SHOOT;
    spindexerManualEnabled = false;
    spindexerManualReverse = false;
    this.spindexer = spindexer;
    
  }

  public boolean isAtVelocity(){
    return targetRPS > 10.0 && Math.abs(shooterLeaderMotor.getVelocity().getValueAsDouble() - targetRPS) < 5.0;
  }

  public double getTargetRPS() {
    return targetRPS;
  }

  public void triggerSpindexerPulse() {
    spindexerPulseUntilSeconds = Timer.getFPGATimestamp() + kSpindexerPulseSeconds;
  }

  public void dashboard(double rps, boolean spindexerEnabled, boolean spindexerReverse) {
    targetRPS = rps;
    wantedState = WantedState.DASHBOARD;
    spindexerManualEnabled = spindexerEnabled;
    spindexerManualReverse = spindexerReverse;
  }

  public void setSpindexerManualEnabled(boolean enabled) {
    spindexerManualEnabled = enabled;
    if (!enabled) {
      spindexerManualReverse = false;
    }
  }

  public void  setSpindexerManualOverride(boolean enabled, boolean reverse) {
    spindexerManualEnabled = enabled;
    spindexerManualReverse = enabled && reverse;
  }

  private void setSpindexerManualVoltages() {
    double spinnerVoltage = spindexerManualReverse ? -7.01:7.01 ;
    double wheelVoltage = spindexerManualReverse ? 12 : -12;
    spindexerSpinnerMotor.setVoltage(spinnerVoltage);
    spindexerWheelMotor.setVoltage(wheelVoltage);
  }
}
