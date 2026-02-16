package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.RobotState;

import org.littletonrobotics.junction.Logger;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

/**
 * Feeder subsystem for FRC 2026 REBUILT.
 *
 * <p>The feeder is the final stage before the shooter flywheels. It takes
 * balls from the spindexer hopper and pushes them into the flywheel
 * contact zone one at a time.
 *
 * <h2>Ball path position:</h2>
 * Ground → Intake Roller → Spindexer → <b>Feeder</b> → [Beam Break] → Shooter Flywheels → Hub
 *
 * <h2>Beam break sensor:</h2>
 * The beam break is located <b>between the feeder and the shooter flywheels</b>.
 * When a ball triggers the beam break, it has already passed through the feeder
 * and is entering the flywheels — meaning it is <b>definitely being shot</b>.
 * This makes the beam break a reliable shot counter (no false positives).
 *
 * <h2>Speed control:</h2>
 * Feeder speed is distance-based (from the shooting lookup table):
 * <ul>
 *   <li>Close range → lower flywheel speed → <b>slower feed</b> to avoid jamming</li>
 *   <li>Far range → higher flywheel speed → <b>faster feed</b>, flywheel absorbs quickly</li>
 * </ul>
 *
 * <h2>Feed gating:</h2>
 * The feeder only runs when:
 * <ol>
 *   <li>Feed is requested (driver holds shoot button / auto command)</li>
 *   <li>The shooter reports {@code isReadyToShoot()} (flywheel + hood + turret locked)</li>
 * </ol>
 *
 * <h2>Hardware:</h2>
 * 1× Kraken X60 (CAN {@value frc.robot.Constants.FeederConstants#kFeederMotorId})
 */
public class Feeder extends SubsystemBase {

    // ==================== SINGLETON ====================
    private static Feeder instance;

    public static void initialize(BooleanSupplier shooterReadySupplier, DoubleSupplier shooterRPSSupplier) {
        if (instance != null) throw new IllegalStateException("Feeder already initialized.");
        instance = new Feeder(shooterReadySupplier, shooterRPSSupplier);
    }

    public static Feeder getInstance() {
        if (instance == null) throw new IllegalStateException("Feeder not initialized. Call initialize() first.");
        return instance;
    }

    public static void resetInstance() { instance = null; }

    private final TalonFX feederMotor;
    // Explicit DutyCycleOut at 50Hz control frame rate (default .set() uses 100Hz).
    // Feeder is a simple duty cycle motor — 50Hz control is plenty responsive.
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withUpdateFreqHz(50);

    // ==================== BEAM BREAK SENSOR ====================
    // Located between the feeder and the shooter flywheels.
    // When a ball triggers this sensor, it has entered the flywheels
    // and is definitely being shot — reliable shot counter.
    private final DigitalInput beamBreak;
    private boolean lastBeamBroken = false;
    private boolean ballPassedThisCycle = false;

    // External suppliers — wired in constructor
    private final BooleanSupplier shooterReadySupplier;
    private final DoubleSupplier shooterRPSSupplier;

    // State
    private boolean feedRequested = false;
    private double targetFeederSpeed = 0.0;

    // ==================== STALL / JAM DETECTION ====================
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private int stallCycleCount = 0;
    private boolean jammed = false;
    private double unjamStartTime = 0;
    private boolean healthy = true;
    private static final int kMaxConfigRetries = 5;

    /**
     * @param shooterReadySupplier   Supplies whether the shooter is locked on
     *                                (Shooter::isReadyToShoot).
     * @param shooterRPSSupplier     Supplies the shooter's target flywheel RPS
     *                                (Shooter::getTargetFlywheelRPS) for
     *                                ratio-based feeder speed.
     */
    private Feeder(BooleanSupplier shooterReadySupplier, DoubleSupplier shooterRPSSupplier) {
        this.shooterReadySupplier = shooterReadySupplier;
        this.shooterRPSSupplier = shooterRPSSupplier;

        feederMotor = new TalonFX(FeederConstants.kFeederMotorId);
        beamBreak = new DigitalInput(FeederConstants.kBeamBreakDIOPort);
        configureMotor();

        // Cache status signals for stall detection
        velocitySignal = feederMotor.getVelocity();
        statorCurrentSignal = feederMotor.getStatorCurrent();
        // 20 Hz — stall detection uses cycle counting with high thresholds, doesn't need 50Hz
        velocitySignal.setUpdateFrequency(20);
        statorCurrentSignal.setUpdateFrequency(20);
        feederMotor.optimizeBusUtilization();
    }

    /** Retry loop for config. */
    private void applyConfig(java.util.function.Supplier<StatusCode> apply, String device) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < kMaxConfigRetries; i++) {
            status = apply.get();
            if (status.isOK()) return;
        }
        healthy = false;
        DriverStation.reportError("[Feeder] " + device + " config FAILED: " + status, false);
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = FeederConstants.kFeederCurrentLimit;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        applyConfig(() -> feederMotor.getConfigurator().apply(config), "Feeder");
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();

        // Batch refresh all status signals ONCE per cycle (no synchronous .refresh())
        com.ctre.phoenix6.BaseStatusSignal.refreshAll(velocitySignal, statorCurrentSignal);
        double velocity = Math.abs(velocitySignal.getValueAsDouble());
        double current = statorCurrentSignal.getValueAsDouble();

        // ---- Beam break edge detection ----
        // Beam break is between feeder and shooter flywheels.
        // When triggered, the ball is entering the flywheels — it is definitely being shot.
        // Beam break returns false when broken (beam interrupted by ball).
        boolean beamBroken = !beamBreak.get();
        ballPassedThisCycle = (!lastBeamBroken && beamBroken); // rising edge = ball entered beam
        lastBeamBroken = beamBroken;

        // ---- Auto-unjam: reverse briefly if jammed ----
        if (jammed) {
            feederMotor.setControl(dutyCycleRequest.withOutput(FeederConstants.kFeederUnjamReverseSpeed));
            if (now - unjamStartTime >= FeederConstants.kFeederUnjamReverseDuration) {
                jammed = false;
                stallCycleCount = 0;
            }
            Logger.recordOutput("Feeder/Jammed", true);
            RobotState.getInstance().setFeederHealthy(healthy);
            return;
        }

        // ---- Stall detection ----
        if (feedRequested && targetFeederSpeed > 0.1) {
            if (current > FeederConstants.kFeederStallCurrentThreshold
                    && velocity < FeederConstants.kFeederStallVelocityThreshold) {
                stallCycleCount++;
            } else {
                stallCycleCount = Math.max(0, stallCycleCount - 2); // decay
            }

            if (stallCycleCount >= FeederConstants.kFeederStallCycleThreshold) {
                jammed = true;
                unjamStartTime = now;
                DriverStation.reportWarning("[Feeder] JAM DETECTED — auto-reversing", false);
            }
        } else {
            stallCycleCount = 0;
        }

        // ---- Feed logic ----
        if (feedRequested && shooterReadySupplier.getAsBoolean()) {
            // Feeder duty = ratio × (shooterRPS / maxRPS)
            // Scales proportionally with shooter — no separate interpolation table needed.
            double shooterRPS = shooterRPSSupplier.getAsDouble();
            targetFeederSpeed = MathUtil.clamp(
                    FeederConstants.kFeederSpeedRatio * (shooterRPS / ShooterConstants.kMaxFlywheelRPS),
                    0.0, 1.0);
            feederMotor.setControl(dutyCycleRequest.withOutput(targetFeederSpeed));
        } else {
            targetFeederSpeed = 0.0;
            feederMotor.setControl(dutyCycleRequest.withOutput(0));
        }

        // Telemetry (AdvantageKit only)
        Logger.recordOutput("Feeder/Speed", targetFeederSpeed);
        Logger.recordOutput("Feeder/FeedRequested", feedRequested);
        Logger.recordOutput("Feeder/Feeding", feedRequested && shooterReadySupplier.getAsBoolean());
        Logger.recordOutput("Feeder/Jammed", jammed);
        Logger.recordOutput("Feeder/StallCycles", stallCycleCount);
        Logger.recordOutput("Feeder/Current", current);
        Logger.recordOutput("Feeder/BeamBroken", beamBroken);
        Logger.recordOutput("Feeder/BallPassed", ballPassedThisCycle);

        RobotState.getInstance().setFeederHealthy(healthy);
    }

    // ==================== FEED CONTROL ====================

    /** @return stator current in amps (for ball presence estimation) */
    public double getCurrent() {
        return statorCurrentSignal.getValueAsDouble();
    }

    /** Request the feeder to run (gated by shooter readiness in periodic). */
    public void requestFeed() {
        feedRequested = true;
    }

    /** Stop requesting feed. */
    public void cancelFeed() {
        feedRequested = false;
    }

    /** @return true if a feed has been requested (may not be actively feeding yet) */
    public boolean isFeedRequested() {
        return feedRequested;
    }

    /** @return the current target feeder duty cycle (used by spindexer to derive its speed) */
    public double getTargetSpeed() {
        return targetFeederSpeed;
    }

    /** @return true if feeder is actively feeding balls */
    public boolean isFeeding() {
        return feedRequested && shooterReadySupplier.getAsBoolean();
    }

    /**
     * @return true if the beam break detected a ball passing through this cycle.
     * Since the beam break is between the feeder and shooter flywheels,
     * a triggered beam break means the ball is definitely being shot.
     * Use for ball counting — call once per cycle from Superstructure.
     */
    public boolean didBallPassThisCycle() {
        return ballPassedThisCycle;
    }

    /** @return true if the beam break is currently broken (ball present) */
    public boolean isBeamBroken() {
        return !beamBreak.get();
    }

    /** Reverse the feeder motor (for unjamming via Superstructure). */
    public void reverse() {
        feedRequested = false;
        feederMotor.setControl(dutyCycleRequest.withOutput(FeederConstants.kFeederUnjamReverseSpeed));
    }

    /**
     * Run feeder at a specific duty cycle for pre-feed reverse.
     * Gentler than full unjam reverse — just creates ball clearance.
     */
    public void setPreFeedReverse(double dutyCycle) {
        feedRequested = false;
        feederMotor.setControl(dutyCycleRequest.withOutput(dutyCycle));
    }

    /** Stop the feeder motor immediately. */
    public void stop() {
        feedRequested = false;
        feederMotor.setControl(dutyCycleRequest.withOutput(0));
    }

    // ==================== COMMAND FACTORIES ====================

    /**
     * <b>Teleop feed command.</b> Bind with {@code whileTrue()} alongside shooter + spindexer.
     *
     * <p>While held: requests feed → feeder runs when shooter is ready at
     * the correct distance-based speed.
     * <p>On release: feeder stops.
     */
    public Command feedCommand() {
        return Commands.startEnd(
                this::requestFeed,
                this::cancelFeed,
                this
        ).withName("Feed");
    }

    /**
     * Autonomous feed command with timeout.
     *
     * @param timeoutSeconds how long to keep feeding
     */
    public Command autoFeedCommand(double timeoutSeconds) {
        return Commands.startEnd(
                this::requestFeed,
                this::cancelFeed,
                this
        ).withTimeout(timeoutSeconds).withName("Auto Feed");
    }
}
