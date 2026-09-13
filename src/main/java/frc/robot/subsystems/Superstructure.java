package frc.robot.subsystems;


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.BooleanSubscriber;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.swerve.SwerveRequest;
import frc.robot.Constants;
import frc.robot.subsystems.Hood.Hood;
import frc.robot.subsystems.Hood.Hood.CurrentState;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.util.LookUpTable;
import frc.robot.util.LookUpTablePass;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;



public class Superstructure extends SubsystemBase{

  public enum SuperstructureWantedState {
    IDLE,
    DRIVING,
    INTAKING,
    DASHBOARD,
    SHOOTING,
    REVUP,
    PRE_SHOOT,
    PASSING,
    REVERSE_INTAKE,
    INTAKE_HOLD

  }
  public enum SuperstructureCurrentState {
    IDLE,
    DRIVING,
    INTAKING,
    DASHBOARD,
    SHOOTING,
    REVUP,
    PRE_SHOOT,
    PASSING,
    REVERSE_INTAKE,
    INTAKE_HOLD
  }
    public CommandSwerveDrivetrain drive;
    public Shooter shooter;
    public Hood hood;
    public Intake intake;
    
    public SuperstructureWantedState wantedState = SuperstructureWantedState.DRIVING;
    public static SuperstructureCurrentState currentState = SuperstructureCurrentState.DRIVING;
    private SuperstructureCurrentState previousCurrentState = SuperstructureCurrentState.DRIVING;
    private Pose2d autoShootSavedPose = null;
    private double targetDistanceMeters = 0.0;
    private static final double kShotIntervalSeconds = 0.2;
    private double lastShotTimestamp = 0.0;
    private static final double kShotExitHeightMeters = 0.6;
    private static final double kAimToleranceDeg = 8;
    private static final double kAimTolerancePassingDeg = 12;
    private static final double kIntakeApproachKp = 1.5;
    private static final double kIntakeApproachMaxSpeedMps = 1.5;
    private static final double kIntakeStopDistanceMeters = 0.15;
    private static final double kIntakeAssistKp = 2.0;
    private static final double kIntakeAssistMaxSpeedMps = 1.1;
    private static final double kIntakeAssistMinDriverSpeedMps = 0.25;
    private static final double kIntakeAssistForwardDistanceMeters = 1.3;
    private static final double kIntakeAssistMaxInterceptTimeSeconds = 1.2;
    private static final double kIntakeAssistMaxOmegaRadPerSec = Units.degreesToRadians(120.0);
    private static final double kIntakeAssistMinTowardDot = 0.25;
    private static final double kIntakeAssistDecelLimitMps2 = 3.0;
    private static final double kRpmToRps = 1.0 / 60.0;
    private static final double kManualHoodAngleDeg = 0.7;
    private static final double kManualShooterSetpointRps = 40.0;
    private static final double kBlueHoodOffsetDeg = 0.419;
    private static final double kShooterWheelRadiusMeters = Units.inchesToMeters(2);
    private static final double kFunnelingMinX = 5.8;
    private static final double kFunnelingMaxX = 10.5;
    private static final double kTrenchHoodDropRadiusMeters = 0.8;
    private static final double kFieldLengthMeters = Constants.FieldConstants.FIELD_LENGTH.in(Meters);
    private static final Translation2d kBlueTrench1 = new Translation2d(4.642, 7.436);
    private static final Translation2d kBlueTrench2 = new Translation2d(4.622, 0.603);
    private static final Translation2d kRedTrench1 = new Translation2d(11.877, 7.405);
    private static final Translation2d kRedTrench2 = new Translation2d(11.888, 0.675);
    private static final Translation2d kBluePassPoint2 = new Translation2d(2.899, 6.753);
    private static final Translation2d kBluePassPoint1 = new Translation2d(2.899, 1.7);

    public boolean in = false;
    public boolean spin = false;
    private static final double kSpinReverseSeconds = 1.0;
    private boolean lastSpin = false;
    private double spinReverseUntilSeconds = 0.0;

    private final DoubleSubscriber shooterTargetRpmSub =
        NetworkTableInstance.getDefault()
            .getDoubleTopic("/Dashboard/ShooterTargetRPM")
            .subscribe(0.0);
    private final BooleanSubscriber hoodInSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/HoodAngleDeg")
            .subscribe(false);
    private final DoubleSubscriber hoodSetpointDegSub =
        NetworkTableInstance.getDefault()
            .getDoubleTopic("/Dashboard/HoodSetpointDeg")
            .subscribe(0.0);
    private final DoubleSubscriber shooterBoostRpmSub =
        NetworkTableInstance.getDefault()
            .getDoubleTopic("/Dashboard/ShooterBoostRPM")
            .subscribe(0.0);
    private final BooleanSubscriber intakeInSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/IntakeIn")
            .subscribe(false);
    private final BooleanSubscriber spindexerEnabledSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/Spindexer")
            .subscribe(false);
    private final BooleanSubscriber spindexerReverseSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/SpindexerReverse")
            .subscribe(false);
    private final BooleanSubscriber dashboardIntakePulseSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/Intake")
            .subscribe(false);
    private final BooleanSubscriber dashboardFunnlingSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/Funnling")
            .subscribe(false);
    private final BooleanSubscriber dashboardShootPulseSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/Shoot")
            .subscribe(false);
    private final BooleanSubscriber dashboardZeroGyroPulseSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/ZeroGyro")
            .subscribe(false);
    private final BooleanSubscriber dashboardDriveModePulseSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/DriveMode")
            .subscribe(false);
    private final BooleanSubscriber dashboardConfirmPulseSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/Confirm")
            .subscribe(false);
    private final BooleanSubscriber dashboardManualSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/Dashboard/Manual")
            .subscribe(false);
    private final BooleanSubscriber passingHumanPlayerSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/1771 passing/Human Player")
            .subscribe(false);
    private final BooleanSubscriber passingDepotSub =
        NetworkTableInstance.getDefault()
            .getBooleanTopic("/1771 passing/Depot")
            .subscribe(false);
    private final StringSubscriber passingSelectedTargetSub =
        NetworkTableInstance.getDefault()
            .getStringTopic("/1771 passing/Selected")
            .subscribe("");
    private final StringSubscriber passingAreaSub =
        NetworkTableInstance.getDefault()
            .getStringTopic("/1771 passing/Area")
            .subscribe("");
    private final StringSubscriber dashboardPassingTargetSub =
        NetworkTableInstance.getDefault()
            .getStringTopic("/Dashboard/PassingTarget")
            .subscribe("");
    private final StringSubscriber dashboardPassingAreaSub =
        NetworkTableInstance.getDefault()
            .getStringTopic("/Dashboard/PassingArea")
            .subscribe("");
    private double dashboardShooterRpm = 0.0;
    private double dashboardShooterBoostRpm = 0.0;
    private boolean dashboardIntakeIn = false;
    private boolean dashboardHoodIn = false;
    private double dashboardHoodSetpointDeg = 0.0;
    private boolean dashboardSpindexerEnabled = false;
    private boolean dashboardSpindexerReverse = false;
    private boolean dashboardFunnling = false;
    private boolean dashboardManual = false;
    private boolean forceShoot = false;
    private String dashboardPassingSelection = "";
    private final LookUpTable lookUpTable = new LookUpTable();

    private enum PassingTargetChoice {
        HUMAN_PLAYER,
        DEPOT
    }

    private final SwerveRequest.RobotCentric intakeRequest =
        new SwerveRequest.RobotCentric().withDeadband(0.0).withRotationalDeadband(0.0);
    @AutoLog
    public static class SuperstructureInputs {
    public String currentState = "IDLE";
    public String wantedState = "IDLE";
    public double targetDistanceMeters = 0.0;
    public double passingDistanceMeters = 0.0;
    public String passingTarget = "None";
    }



 

    @AutoLog
     public static class DriveTrainInputs {
    public Pose2d pose =  new Pose2d();
    public ChassisSpeeds speed = new ChassisSpeeds();
    public double xVelocity = 0;
    public double yVelocity = 0;
    public double thetaVelocity = 0;
    public Pose2d[] trajectory = new Pose2d[0];
    public Pose2d trajectorySetpoint = new Pose2d();
  }

  private final DriveTrainInputsAutoLogged driveInputs = new DriveTrainInputsAutoLogged();
  private final SuperstructureInputsAutoLogged superstructureInputs = new SuperstructureInputsAutoLogged();

  private Pose2d[] latestTrajectory = new Pose2d[0];
  private Pose2d latestTrajectorySetpoint = new Pose2d();

  public void setTrajectory(Pose2d[] trajectory) {
      latestTrajectory = trajectory;
  }

  public void setTrajectorySetpoint(Pose2d setpoint) {
      latestTrajectorySetpoint = setpoint;
  }


  public Superstructure(CommandSwerveDrivetrain drive, Shooter shooter, Hood hood, Intake intake){ {
    this.drive = drive;
    this.shooter = shooter;
    this.hood = hood;
    this.intake = intake;


  }
}

 @Override
    public void periodic() {
        Logger.recordOutput("RobotPose", drive.getPose());
        Logger.recordOutput("MatchTime", DriverStation.getMatchTime());
        Logger.recordOutput("AutoWinner", DriverStation.getGameSpecificMessage());
        Logger.recordOutput("Battery", RobotController.getBatteryVoltage());

        updateInputs();
        updateDashboardControls();
        


        Logger.processInputs("Superstructure", superstructureInputs);
        Logger.processInputs("DriveTrain", driveInputs);
        Logger.recordOutput("Aimed at the hub", isAimedAtHub());
        handleStates();
        applyStates();
        handleAutoShootOdometry();


  }


    private void updateInputs() {
        driveInputs.xVelocity = drive.getChassisSpeeds().vxMetersPerSecond;
        driveInputs.yVelocity =  drive.getChassisSpeeds().vyMetersPerSecond;
        driveInputs.speed = drive.getChassisSpeeds();
        driveInputs.thetaVelocity = drive.getChassisSpeeds().omegaRadiansPerSecond;
        driveInputs.pose = drive.getPose();
        driveInputs.trajectory = latestTrajectory;
        driveInputs.trajectorySetpoint = latestTrajectorySetpoint;
        superstructureInputs.currentState = currentState.toString();
        superstructureInputs.wantedState = wantedState.toString();
         Translation2d hubTarget =
            currentState == SuperstructureCurrentState.PASSING || isInFunnelingXRange()
                ? getPassingTarget()
                : DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue) == DriverStation.Alliance.Red
                    ? Constants.FieldConstants.HUB_RED.toTranslation2d()
                    : Constants.FieldConstants.HUB_BLUE.toTranslation2d();
        superstructureInputs.targetDistanceMeters =
            drive.getPose().getTranslation().getDistance(hubTarget);
        superstructureInputs.passingDistanceMeters = getPassingDistanceMeters();
        superstructureInputs.passingTarget = getPassingTargetName();

        Logger.recordOutput("Passing/DistanceMeters", superstructureInputs.passingDistanceMeters);
        Logger.recordOutput("Passing/Target", superstructureInputs.passingTarget);

    }
   

    private void updateDashboardControls() {
        double shooterRpm = shooterTargetRpmSub.get();
        for (var sample : shooterTargetRpmSub.readQueue()) {
            shooterRpm = sample.value;
        }
        dashboardShooterRpm = shooterRpm;

        double boostRpm = shooterBoostRpmSub.get();
        for (var sample : shooterBoostRpmSub.readQueue()) {
            boostRpm = sample.value;
        }
        dashboardShooterBoostRpm = boostRpm;

        boolean hoodIn = hoodInSub.get();
        for (var sample : hoodInSub.readQueue()) {
            hoodIn = sample.value;
        }
        dashboardHoodIn = hoodIn;

        double hoodSetpointDeg = hoodSetpointDegSub.get();
        for (var sample : hoodSetpointDegSub.readQueue()) {
            hoodSetpointDeg = sample.value;
        }
        dashboardHoodSetpointDeg = hoodSetpointDeg;

        boolean intakeIn = intakeInSub.get();
        for (var sample : intakeInSub.readQueue()) {
            intakeIn = sample.value;
        }
        dashboardIntakeIn = intakeIn;

        boolean spindexerEnabled = spindexerEnabledSub.get();
        for (var sample : spindexerEnabledSub.readQueue()) {
            spindexerEnabled = sample.value;
        }
        dashboardSpindexerEnabled = spindexerEnabled;

        boolean spindexerReverse = spindexerReverseSub.get();
        for (var sample : spindexerReverseSub.readQueue()) {
            spindexerReverse = sample.value;
        }
        dashboardSpindexerReverse = spindexerReverse;

        boolean funnling = dashboardFunnlingSub.get();
        for (var sample : dashboardFunnlingSub.readQueue()) {
            funnling = sample.value;
        }
        dashboardFunnling = funnling;

        boolean manual = dashboardManualSub.get();
        for (var sample : dashboardManualSub.readQueue()) {
            manual = sample.value;
        }
        dashboardManual = manual;

        boolean shootPulse = consumePulse(dashboardShootPulseSub);
        boolean intakePulse = consumePulse(dashboardIntakePulseSub);
        boolean drivePulse = consumePulse(dashboardDriveModePulseSub);
        boolean zeroGyroPulse = consumePulse(dashboardZeroGyroPulseSub);
        boolean confirmPulse = consumePulse(dashboardConfirmPulseSub);

        if (zeroGyroPulse) {
            drive.zeroGyro();
        }
        if (drivePulse) {
            wantedState = SuperstructureWantedState.DRIVING;
        }
        if (intakePulse) {
            wantedState = SuperstructureWantedState.INTAKING;
        }
        if (shootPulse) {
            wantedState = SuperstructureWantedState.PRE_SHOOT;
        }
        dashboardPassingSelection = getLatestPassingSelection();
    }

    private void handleStates() {
        switch (wantedState) {
            case IDLE:
            
                currentState = SuperstructureCurrentState.IDLE;
                
                break;
            case DRIVING:
            if(ifRevUP()){
                currentState = SuperstructureCurrentState.REVUP;
            }
            else{                
                currentState = SuperstructureCurrentState.DRIVING;
        }
            
                                currentState = SuperstructureCurrentState.DRIVING;

                break;
            case INTAKING:
                currentState = SuperstructureCurrentState.INTAKING;
                break;
            case DASHBOARD:
                currentState = SuperstructureCurrentState.DASHBOARD;
                break;
            case SHOOTING:
                if (forceShoot || shootCheck()) {
                    if (isInFunnelingXRange()) {
                        currentState = SuperstructureCurrentState.PASSING;
                    } else {
                        currentState = SuperstructureCurrentState.SHOOTING;
                    }
                } else {
                    currentState = isInFunnelingXRange()
                        ? SuperstructureCurrentState.PASSING
                        : SuperstructureCurrentState.PRE_SHOOT;

                }


                break;
            case PRE_SHOOT:
            currentState = (dashboardFunnling || isInFunnelingXRange())
                ? SuperstructureCurrentState.PASSING
                : SuperstructureCurrentState.PRE_SHOOT;
            break;
            case PASSING:
            currentState = SuperstructureCurrentState.PASSING;
            break;
            case REVERSE_INTAKE:
            currentState = SuperstructureCurrentState.REVERSE_INTAKE;
            break;
            case INTAKE_HOLD:
            currentState = SuperstructureCurrentState.INTAKE_HOLD;
            break;
        }
    }

    private double getRevUpPose(){
        Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Blue);
        if(alliance == Alliance.Red){
            return 5.7;
        }
        else{
            return 11.187;
        }
    }
    
    private boolean ifRevUP(){
        double revUpPose = getRevUpPose();
        if((drive.getPose().getX() > revUpPose && isRedAlliance())
            || (drive.getPose().getX() < revUpPose && !isRedAlliance())){
                return true;
            }
        else{
            return false;
        }
    }

    private boolean consumePulse(BooleanSubscriber subscriber) {
        boolean triggered = false;
        for (var sample : subscriber.readQueue()) {
            if (sample.value) {
                triggered = true;
            }
        }
        return triggered;
    }

    public static SuperstructureCurrentState getCurrentState() {
        return currentState;
    }

    public boolean shootCheck(){
        if (shooter.isAtVelocity()
            && hood.isAtTargetAngle(hood.getTargetAngleDegrees())
            && (dashboardManual || isAimedAtHub())
            && drive.getChassisSpeeds().omegaRadiansPerSecond < Units.degreesToRadians(5)) {
            return true;

        }
        else{
            return false;
        }
        
    }

    private void applyStates() {
        if (currentState != SuperstructureCurrentState.PRE_SHOOT) {
            lastSpin = spin;
        }
        if (currentState != SuperstructureCurrentState.PRE_SHOOT
            && currentState != SuperstructureCurrentState.PASSING) {
        }
        switch (currentState) {
            case IDLE:
            
            case DRIVING:
                shooter.setOff();
                    hood.setIdle();
                
                if (dashboardIntakeIn || in) {
                    intake.goHome();
                }
                shooter.setSpindexerManualEnabled(false);

                
                break;
            case INTAKING:
                shooter.setOff();
                hood.setIdle();
                intake.startIntake();
                shooter.setSpindexerManualEnabled(false);
                break;
            case DASHBOARD:
                shooter.dashboard(
                    (dashboardShooterRpm + dashboardShooterBoostRpm) * kRpmToRps,
                    dashboardSpindexerEnabled,
                    dashboardSpindexerReverse);
                if (dashboardHoodIn) {
                    hood.setIdle();
                } else {
                    hood.moveToAngle(dashboardHoodSetpointDeg);
                }
                if(dashboardIntakeIn || in){
                    intake.goHome();
                
                }
                break;
            case SHOOTING:
                shootingPipeline();
                break;

            case REVUP:
            shooter.preShoot(16.6666666667 ,false);
            hood.setIdle();
         if (dashboardIntakeIn || in) {
            intake.goHome();
        }      
            shooter.setSpindexerManualEnabled(false);
            break;

            case PRE_SHOOT:
            double preShootVelocityRps = updateShotSetpointsAndGetShooterVelocityRps();
            shooter.preShoot(preShootVelocityRps, false);
               if (dashboardIntakeIn || in) {   
            intake.goHome();
        }
            shooter.setSpindexerManualEnabled(false);
             break;

        case PASSING:
        double passVelocityRps = updatePassSetpointsAndGetShooterVelocityRps();

        if (shooter.isAtVelocity() && isAimedAtHub(kAimTolerancePassingDeg)) {
            shooter.preShoot(passVelocityRps, true);
            intake.shoot();
        } else {
            shooter.preShoot(passVelocityRps, false);
        }

        break;
        case REVERSE_INTAKE:
        shooter.setOff();
        hood.setIdle();
        intake.reverse();
        shooter.setSpindexerManualOverride(false, false);

        break;

        case INTAKE_HOLD:
        if (dashboardFunnling || isInFunnelingXRange()) {
            double holdPassVelocityRps = updatePassSetpointsAndGetShooterVelocityRps();
            if (shooter.isAtVelocity() && isAimedAtHub(kAimTolerancePassingDeg)) {
                shooter.preShoot(holdPassVelocityRps, true);
            } else {
                shooter.preShoot(holdPassVelocityRps, false);
            }
        } else {
            double intakeHoldVelocityRps = updateShotSetpointsAndGetShooterVelocityRps();
            if (shooter.isAtVelocity() && isAimedAtHub()) {
                shooter.shoot(intakeHoldVelocityRps);
            } else {
                shooter.preShoot(intakeHoldVelocityRps, false);
            }
        }
        intake.intakeHold();
        shooter.setSpindexerManualEnabled(false);
        break;

        }

        if (isNearTrench()) {
            hood.setIdle();
        }



    }

    public void shootingPipeline(){
        double boostedShooterVelocity = updateShotSetpointsAndGetShooterVelocityRps();

        Logger.recordOutput("boosted", boostedShooterVelocity);
        shooter.shoot(boostedShooterVelocity);

        boolean forceIntakeHome = dashboardIntakeIn || in;
        if (forceIntakeHome) {
            intake.goHome();
        } else {
            intake.shoot();
        }

        double now = Timer.getFPGATimestamp();
        if (spin && !lastSpin) {
            spinReverseUntilSeconds = now + kSpinReverseSeconds;
        }
        lastSpin = spin;
        if (spin) {
            boolean reverse = now < spinReverseUntilSeconds;
            shooter.setSpindexerManualOverride(true, reverse);
        } else if (dashboardSpindexerReverse) {
            shooter.setSpindexerManualOverride(true, true);
        } else {
            shooter.setSpindexerManualEnabled(false);
        }

                if(shooter.isAtVelocity()&& (dashboardManual || isAimedAtHub()) && drive.getChassisSpeeds().omegaRadiansPerSecond< Units.degreesToRadians(2)){
                    spawnFuelShot();
                }
            }
        
    


    public void requestIdle() {
        wantedState = SuperstructureWantedState.IDLE;
    }

    public void requestIntake() {
        wantedState = SuperstructureWantedState.INTAKING;
    }
    public void requestReverseIntake() {
        wantedState = SuperstructureWantedState.REVERSE_INTAKE;
    }

    public void requestShooting() {
        wantedState = SuperstructureWantedState.SHOOTING;
    }
      public void requestShootingPRE() {
        wantedState = SuperstructureWantedState.PRE_SHOOT;
    }

    public void setTargetDistanceMeterps() {
        wantedState = SuperstructureWantedState.PRE_SHOOT;
    }

    public void setTargetDistanceMeters(double distanceMeters) {
        targetDistanceMeters = distanceMeters;
    }
    
    public void intakeIn(boolean in){
        this.in = in;
    }
       
    public void spindexer(boolean spin){
        this.spin = spin;
    }

 public Command spin(boolean spinn){
        return Commands.runOnce(()->{
            spindexer(spinn);
        } );
    }
    public Command goIn(boolean in){
        return Commands.runOnce(()->{
            intakeIn(in);
        } );
    }
    public Command setIntake(){
        return Commands.runOnce(() -> {
            in = false;
            requestIntake();
        });
    }
    public Command setHome() {
        return Commands.runOnce(() -> {
            intake.goHome();
    });
}
    public Command setIdle(){
        return Commands.runOnce(() -> {
            requestIdle();
        });
        }
    public Command setShooting(){
        return Commands.runOnce(() -> {
            requestShooting();
        });
    }
    public Command setPreshoot(){
        return Commands.runOnce(() -> {
            requestShootingPRE();
        });
    }
    public Command setDriving(){
        return Commands.runOnce(() -> {
            wantedState = SuperstructureWantedState.DRIVING;
            forceShoot = false;
        });
    }

    public Command setForceShoot(boolean force) {
        return Commands.runOnce(() -> forceShoot = force);
    }

    public Command setReverseIntake() {
        return Commands.runOnce(this::requestReverseIntake);
    }

    public Command setIntakeHold() {
        return Commands.runOnce(() -> wantedState = SuperstructureWantedState.INTAKE_HOLD);
    }

    public Command setPassing() {
        return Commands.runOnce(() -> {
            wantedState = SuperstructureWantedState.PASSING;
        });
    }

    public void spawnFuelShot() {
        if (!RobotBase.isSimulation()) {
            return;
        }
        if (currentState != SuperstructureCurrentState.SHOOTING) {
            return;
        }
        if (!shooter.isAtVelocity()) {
            return;
        }
        if (!isAimedAtHub()) {
            return;
        }
        double now = Timer.getFPGATimestamp();
        if (now - lastShotTimestamp < kShotIntervalSeconds) {
            return;
        }

        Pose2d robotPose = drive.getPose();
        Translation2d shooterOffset = new Translation2d(0.45, 0.0);
        ChassisSpeeds robotFieldSpeeds = drive.getChassisSpeeds();
        Rotation2d shooterFacing = robotPose.getRotation();
        double hoodAngleDegrees = hood.getTargetAngleDegrees();
        double shooterRPS = shooter.getTargetRPS(); //rps
        double shooterAngualrVelocity = shooterRPS * (2*Math.PI);
        

        double shooterSpeedMetersPerSecond =
            shooterAngualrVelocity * kShooterWheelRadiusMeters;

      
        lastShotTimestamp = now;
    }

    private Translation2d getHubDirection() {
        Pose2d robotPose = drive.getPose();
  Translation2d disntance = new Translation2d() ;
        if (DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red) {
            disntance = Constants.FieldConstants.HUB_RED.toTranslation2d().minus(robotPose.getTranslation());

        }
        else{
            disntance = Constants.FieldConstants.HUB_BLUE.toTranslation2d().minus(robotPose.getTranslation());


        }        
     
        if (disntance.getNorm() < 1e-6) {
            return new Translation2d(1.0, 0.0);
        }
        return disntance.div(disntance.getNorm());
    }

    private boolean isRedAlliance() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
    }

    private boolean isInFunnelingXRange() {
        double x = drive.getPose().getX();
        return x > kFunnelingMinX && x < kFunnelingMaxX;
    }

    private boolean isNearTrench() {
        Translation2d robotTranslation = drive.getPose().getTranslation();
        if (isRedAlliance()) {
            return robotTranslation.getDistance(kRedTrench1) <= kTrenchHoodDropRadiusMeters
                || robotTranslation.getDistance(kRedTrench2) <= kTrenchHoodDropRadiusMeters;
        }
        return robotTranslation.getDistance(kBlueTrench1) <= kTrenchHoodDropRadiusMeters
            || robotTranslation.getDistance(kBlueTrench2) <= kTrenchHoodDropRadiusMeters;
    }

    private double getHubDistanceMeters() {
        Translation2d target =
            isRedAlliance()
                ? Constants.FieldConstants.HUB_RED.toTranslation2d()
                : Constants.FieldConstants.HUB_BLUE.toTranslation2d();
        return drive.getPose().getTranslation().getDistance(target);
    }

    private double updateShotSetpointsAndGetShooterVelocityRps() {
        if (dashboardManual) {
            hood.moveToAngle(kManualHoodAngleDeg);
            return kManualShooterSetpointRps;
        }

        LookUpTable.LookUpTableTest lookUpTableTest = lookUpTable.LookUpTableOutput(getHubDistanceMeters());
        double hoodDegrees = lookUpTableTest.getAngle();
        if (!isRedAlliance()) {
            hoodDegrees -= kBlueHoodOffsetDeg;
        }
        hood.moveToAngle(hoodDegrees);
        return (lookUpTableTest.getRPS() / 60) + (dashboardShooterBoostRpm * kRpmToRps);
    }

    private double updatePassSetpointsAndGetShooterVelocityRps() {
        LookUpTablePass.LookUpTableTest passLookUpTableTest =
            LookUpTablePass.LookUpTableOutput(getPassingDistanceMeters());
        hood.moveToAngle(passLookUpTableTest.getAngle());
        return (passLookUpTableTest.getRPM() / 60.0) + (dashboardShooterBoostRpm * kRpmToRps) + (-400 * kRpmToRps);
    }

    public Rotation2d getRotationToPoint( Translation2d fieldTarget) {
        Pose2d robotPose = drive.getPose();
        Translation2d delta = fieldTarget.minus(robotPose.getTranslation());
        if (delta.getNorm() < 1e-6) {
            return robotPose.getRotation();
        }
        return new Rotation2d(delta.getX(), delta.getY());
    }


    public Rotation2d getHubHeading() {
        boolean usePassingTarget =
            currentState == SuperstructureCurrentState.PASSING || isInFunnelingXRange();
        Translation2d target;
        if (usePassingTarget) {
            target = getPassingTarget();
        } else {
            target = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
                ? Constants.FieldConstants.HUB_RED.toTranslation2d()
                : Constants.FieldConstants.HUB_BLUE.toTranslation2d();
        }

        return getRotationToPoint(target).plus(Rotation2d.fromDegrees(0));
    }

    private double getPassingDistanceMeters() {
        return drive.getPose().getTranslation().getDistance(getPassingTarget());
    }

    private Translation2d getPassingTarget() {
        boolean isRedAlliance = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
        Translation2d humanPlayerTarget =
            isRedAlliance ? reflectBluePoseToRedSide(kBluePassPoint1) : kBluePassPoint1;
        Translation2d depotTarget =
            isRedAlliance ? reflectBluePoseToRedSide(kBluePassPoint2) : kBluePassPoint2;
        PassingTargetChoice requestedTarget = getRequestedPassingTargetChoice();

        if (requestedTarget == null) {
            return getClosestPassingTarget(humanPlayerTarget, depotTarget);
        }

        return requestedTarget == PassingTargetChoice.HUMAN_PLAYER
            ? humanPlayerTarget
            : depotTarget;
    }

    private Translation2d reflectBluePoseToRedSide(Translation2d bluePose) {
        return new Translation2d(kFieldLengthMeters - bluePose.getX(), bluePose.getY());
    }

    private String getPassingTargetName() {
        PassingTargetChoice requestedTarget = getRequestedPassingTargetChoice();
        if (requestedTarget == PassingTargetChoice.HUMAN_PLAYER) {
            return "Human Player";
        }
        if (requestedTarget == PassingTargetChoice.DEPOT) {
            return "Depot";
        }
        return "Auto";
    }

    private Translation2d getClosestPassingTarget(Translation2d optionA, Translation2d optionB) {
        Translation2d robotTranslation = drive.getPose().getTranslation();
        double optionADistance = robotTranslation.getDistance(optionA);
        double optionBDistance = robotTranslation.getDistance(optionB);
        return optionADistance <= optionBDistance ? optionA : optionB;
    }

    private String getLatestPassingSelection() {
        String selection = dashboardPassingSelection;
        selection = getLatestStringValue(passingSelectedTargetSub, selection);
        selection = getLatestStringValue(passingAreaSub, selection);
        selection = getLatestStringValue(dashboardPassingTargetSub, selection);
        selection = getLatestStringValue(dashboardPassingAreaSub, selection);
        Logger.recordOutput("Passing/SelectionRaw", selection);
        return selection;
    }

    private String getLatestStringValue(StringSubscriber subscriber, String fallback) {
        String value = subscriber.get();
        for (var sample : subscriber.readQueue()) {
            value = sample.value;
        }
        return value == null || value.isBlank() ? fallback : value.trim();
    }

    private PassingTargetChoice getRequestedPassingTargetChoice() {
        PassingTargetChoice explicitSelection = parsePassingSelection(dashboardPassingSelection);
        if (explicitSelection != null) {
            return explicitSelection;
        }

        boolean requestHumanPlayer = passingHumanPlayerSub.get();
        boolean requestDepot = passingDepotSub.get();
        if (requestHumanPlayer == requestDepot) {
            return null;
        }
        return requestHumanPlayer ? PassingTargetChoice.HUMAN_PLAYER : PassingTargetChoice.DEPOT;
    }

    private PassingTargetChoice parsePassingSelection(String selection) {
        if (selection == null) {
            return null;
        }

        String normalized = selection.trim().toLowerCase();
        if (normalized.isEmpty()) {
            return null;
        }

        if (normalized.equals("human player")
            || normalized.equals("humanplayer")
            || normalized.equals("hp")
            || normalized.equals("player")
            || normalized.equals("area 1")
            || normalized.equals("area1")
            || normalized.equals("1")) {
            return PassingTargetChoice.HUMAN_PLAYER;
        }

        if (normalized.equals("depot")
            || normalized.equals("area 2")
            || normalized.equals("area2")
            || normalized.equals("2")) {
            return PassingTargetChoice.DEPOT;
        }

        return null;
    }

    public boolean isManual() {
        return dashboardManual;
    }

    public boolean isPassing(){
        if (currentState == SuperstructureCurrentState.PASSING || isInFunnelingXRange()) {
            return true;
        }
        return false;
    }

    public boolean isShootingRequested() {
        return wantedState == SuperstructureWantedState.SHOOTING;
    }

    public boolean shouldUseShooterCameraInAuto() {
        return wantedState == SuperstructureWantedState.SHOOTING
            || wantedState == SuperstructureWantedState.PRE_SHOOT
            || currentState == SuperstructureCurrentState.PRE_SHOOT
            || currentState == SuperstructureCurrentState.SHOOTING;
    }

    public ChassisSpeeds applyIntakeAssist(
        ChassisSpeeds fieldRelativeDriverSpeeds,
        double visionLateralErrorMeters,
        boolean hasVisionTarget) {
        Logger.recordOutput("IntakeAssist/Enabled", false);
        if (currentState != SuperstructureCurrentState.INTAKING || !hasVisionTarget) {
            return fieldRelativeDriverSpeeds;
        }

        Rotation2d robotRotation = drive.getPose().getRotation();
        ChassisSpeeds robotRelativeDriverSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            fieldRelativeDriverSpeeds,
            robotRotation);

        double vx = robotRelativeDriverSpeeds.vxMetersPerSecond;
        double vy = robotRelativeDriverSpeeds.vyMetersPerSecond;
        double speed = Math.hypot(vx, vy);
        if (speed < kIntakeAssistMinDriverSpeedMps) {
            return fieldRelativeDriverSpeeds;
        }
        if (Math.abs(fieldRelativeDriverSpeeds.omegaRadiansPerSecond) > kIntakeAssistMaxOmegaRadPerSec) {
            return fieldRelativeDriverSpeeds;
        }

        Translation2d wantedVelocityUnit = new Translation2d(vx / speed, vy / speed);
        Translation2d noteVectorRobot = new Translation2d(
            kIntakeAssistForwardDistanceMeters,
            visionLateralErrorMeters);
        if (noteVectorRobot.getNorm() < 1e-6) {
            return fieldRelativeDriverSpeeds;
        }

        Translation2d noteUnit = noteVectorRobot.div(noteVectorRobot.getNorm());
        double towardDot = wantedVelocityUnit.getX() * noteUnit.getX() + wantedVelocityUnit.getY() * noteUnit.getY();
        if (towardDot < kIntakeAssistMinTowardDot) {
            return fieldRelativeDriverSpeeds;
        }

        double alongDistanceToNote =
            noteVectorRobot.getX() * wantedVelocityUnit.getX()
                + noteVectorRobot.getY() * wantedVelocityUnit.getY();
        if (alongDistanceToNote <= 0.0) {
            return fieldRelativeDriverSpeeds;
        }

        double interceptTimeSeconds = alongDistanceToNote / speed;
        if (interceptTimeSeconds > kIntakeAssistMaxInterceptTimeSeconds) {
            return fieldRelativeDriverSpeeds;
        }

        // Signed perpendicular distance from note vector to wanted velocity direction.
        double perpendicularDistance =
            noteVectorRobot.getX() * wantedVelocityUnit.getY()
                - noteVectorRobot.getY() * wantedVelocityUnit.getX();

        Translation2d perpendicularUnit = new Translation2d(-wantedVelocityUnit.getY(), wantedVelocityUnit.getX());
        double assistSpeed = MathUtil.clamp(
            -perpendicularDistance * kIntakeAssistKp,
            -kIntakeAssistMaxSpeedMps,
            kIntakeAssistMaxSpeedMps);

        double assistedVx = vx + perpendicularUnit.getX() * assistSpeed;
        double assistedVy = vy + perpendicularUnit.getY() * assistSpeed;

        // Limit assist speed so the robot can decelerate before reaching the note.
        double maxTranslationSpeed = Math.sqrt(
            Math.max(0.0, 2.0 * kIntakeAssistDecelLimitMps2 * alongDistanceToNote));
        double assistedSpeed = Math.hypot(assistedVx, assistedVy);
        if (assistedSpeed > maxTranslationSpeed && assistedSpeed > 1e-6) {
            double scale = maxTranslationSpeed / assistedSpeed;
            assistedVx *= scale;
            assistedVy *= scale;
        }

        ChassisSpeeds assistedRobotRelative = new ChassisSpeeds(
            assistedVx,
            assistedVy,
            fieldRelativeDriverSpeeds.omegaRadiansPerSecond);

        ChassisSpeeds assistedFieldRelative = ChassisSpeeds.fromRobotRelativeSpeeds(
            assistedRobotRelative,
            robotRotation);

        Logger.recordOutput("IntakeAssist/Enabled", true);
        Logger.recordOutput("IntakeAssist/VisionLateralMeters", visionLateralErrorMeters);
        Logger.recordOutput("IntakeAssist/PerpDistanceMeters", perpendicularDistance);
        Logger.recordOutput("IntakeAssist/AssistSpeedMps", assistSpeed);
        return assistedFieldRelative;
    }

    

    private void handleAutoShootOdometry() {
        boolean isShootingState = currentState == SuperstructureCurrentState.SHOOTING
            || currentState == SuperstructureCurrentState.PRE_SHOOT;
        boolean wasShootingState = previousCurrentState == SuperstructureCurrentState.SHOOTING
            || previousCurrentState == SuperstructureCurrentState.PRE_SHOOT;

        if (!wasShootingState && isShootingState && DriverStation.isAutonomous()) {
            autoShootSavedPose = drive.getPose();
            Logger.recordOutput("AutoShootOdometry/SavedPose", autoShootSavedPose);
        }

        if (wasShootingState && !isShootingState && autoShootSavedPose != null) {
            Pose2d restored = new Pose2d(autoShootSavedPose.getTranslation(), drive.getPose().getRotation());
            drive.resetPose(restored);
            Logger.recordOutput("AutoShootOdometry/RestoredPose", restored);
            autoShootSavedPose = null;
        }

        previousCurrentState = currentState;
    }

    private boolean isAimedAtHub() {
        return isAimedAtHub(kAimToleranceDeg);
    }

    private boolean isAimedAtHub(double toleranceDeg) {
        double desired = getHubHeading().getRadians() + Math.PI;
        double current = drive.getPose().getRotation().getRadians();
        double error = MathUtil.angleModulus(desired - current);
        Logger.recordOutput("Hub", Math.abs(Units.radiansToDegrees(error)));
        return Math.abs(Units.radiansToDegrees(error)) <= toleranceDeg;
    }

    }    
