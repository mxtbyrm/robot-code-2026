package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.SpindexerConstants;
import frc.robot.RobotState;

import org.littletonrobotics.junction.Logger;

/**
 * Spindexer subsystem for FRC 2026 REBUILT.
 *
 * <p>The spindexer is a rotating hopper that indexes balls from the intake
 * toward the feeder. It sits between the intake roller and the feeder,
 * organizing balls so they feed one-at-a-time into the shooter.
 *
 * <h2>Ball path position:</h2>
 * Ground → Intake Roller → <b>Spindexer</b> → Feeder → Shooter Flywheels → Hub
 *
 * <h2>Speeds:</h2>
 * <ul>
 *   <li><b>Intake speed</b> (slow) — while collecting balls, gently rotates
 *       to make room in the hopper.</li>
 *   <li><b>Feed speed</b> (fast) — while shooting, actively pushes balls
 *       into the feeder.</li>
 *   <li><b>Reverse</b> — unjam / eject.</li>
 * </ul>
 *
 * <h2>Hardware:</h2>
 * 1× Kraken X60 (CAN {@value frc.robot.constants.SpindexerConstants#kSpindexerMotorId})
 */
public class Spindexer extends SubsystemBase {

    // ==================== SINGLETON ====================
    private static Spindexer instance;

    public static void initialize() {
        if (instance != null) throw new IllegalStateException("Spindexer already initialized.");
        instance = new Spindexer();
    }

    public static Spindexer getInstance() {
        if (instance == null) throw new IllegalStateException("Spindexer not initialized. Call initialize() first.");
        return instance;
    }

    public static void resetInstance() { instance = null; }

    private final TalonFX spindexerMotor;
    // Explicit DutyCycleOut at 50Hz control frame rate (default .set() uses 100Hz).
    // Spindexer is a simple duty cycle motor — 50Hz control is plenty responsive.
    private final DutyCycleOut dutyCycleRequest = new DutyCycleOut(0).withUpdateFreqHz(50);

    // ==================== STALL / JAM DETECTION ====================
    private final StatusSignal<AngularVelocity> velocitySignal;
    private final StatusSignal<Current> statorCurrentSignal;
    private int stallCycleCount = 0;
    private boolean jammed = false;
    private double unjamStartTime = 0;
    private double commandedOutput = 0;
    private boolean healthy = true;
    private static final int kMaxConfigRetries = 5;

    private Spindexer() {
        spindexerMotor = new TalonFX(SpindexerConstants.kSpindexerMotorId);
        configureMotor();

        velocitySignal = spindexerMotor.getVelocity();
        statorCurrentSignal = spindexerMotor.getStatorCurrent();
        // 20 Hz — stall detection uses cycle counting with high thresholds, doesn't need 50Hz
        velocitySignal.setUpdateFrequency(20);
        statorCurrentSignal.setUpdateFrequency(20);
        spindexerMotor.optimizeBusUtilization();
    }

    /** Retry loop for config. */
    private void applyConfig(java.util.function.Supplier<StatusCode> apply, String device) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < kMaxConfigRetries; i++) {
            status = apply.get();
            if (status.isOK()) return;
        }
        healthy = false;
        DriverStation.reportError("[Spindexer] " + device + " config FAILED: " + status, false);
    }

    private void configureMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = SpindexerConstants.kSpindexerCurrentLimit;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        applyConfig(() -> spindexerMotor.getConfigurator().apply(config), "Spindexer");
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        double now = Timer.getFPGATimestamp();

        // Batch refresh all status signals ONCE per cycle (no synchronous .refresh())
        com.ctre.phoenix6.BaseStatusSignal.refreshAll(velocitySignal, statorCurrentSignal);
        double velocity = Math.abs(velocitySignal.getValueAsDouble());
        double current = statorCurrentSignal.getValueAsDouble();

        // ---- Auto-unjam: reverse briefly if jammed ----
        if (jammed) {
            spindexerMotor.setControl(dutyCycleRequest.withOutput(SpindexerConstants.kSpindexerUnjamSpeed));
            if (now - unjamStartTime >= SpindexerConstants.kSpindexerUnjamDuration) {
                jammed = false;
                stallCycleCount = 0;
                // Re-apply the last commanded output so the motor doesn't stay reversed
                spindexerMotor.setControl(dutyCycleRequest.withOutput(commandedOutput));
            }
        }

        // ---- Stall detection (only when motor is commanded forward) ----
        if (!jammed && commandedOutput > 0.1) {
            if (current > SpindexerConstants.kSpindexerStallCurrentThreshold
                    && velocity < SpindexerConstants.kSpindexerStallVelocityThreshold) {
                stallCycleCount++;
            } else {
                stallCycleCount = Math.max(0, stallCycleCount - 2);
            }

            if (stallCycleCount >= SpindexerConstants.kSpindexerStallCycleThreshold) {
                jammed = true;
                unjamStartTime = now;
                DriverStation.reportWarning("[Spindexer] JAM DETECTED — auto-reversing", false);
            }
        } else if (!jammed) {
            stallCycleCount = 0;
        }

        // Telemetry (AdvantageKit only)
        Logger.recordOutput("Spindexer/Output", spindexerMotor.get());
        Logger.recordOutput("Spindexer/Jammed", jammed);
        Logger.recordOutput("Spindexer/Current", current);
        Logger.recordOutput("Spindexer/StallCycles", stallCycleCount);

        RobotState.getInstance().setSpindexerHealthy(healthy);
    }

    // ==================== MOTOR CONTROL ====================

    /** @return smoothed stator current in amps (for ball presence estimation) */
    public double getCurrent() {
        return statorCurrentSignal.getValueAsDouble();
    }

    /** @return true if the spindexer is actively being commanded forward */
    public boolean isRunning() {
        return commandedOutput > 0.05;
    }

    /** Run at intake speed (slow — collecting balls into hopper). */
    public void runIntakeSpeed() {
        commandedOutput = SpindexerConstants.kSpindexerIntakeSpeed;
        spindexerMotor.setControl(dutyCycleRequest.withOutput(commandedOutput));
    }

    /** Reverse (unjam). */
    public void reverse() {
        commandedOutput = SpindexerConstants.kSpindexerReverseSpeed;
        spindexerMotor.setControl(dutyCycleRequest.withOutput(commandedOutput));
    }

    /** Run spindexer at arbitrary duty cycle (for pre-feed reverse). */
    public void setSpeed(double dutyCycle) {
        commandedOutput = dutyCycle;
        spindexerMotor.setControl(dutyCycleRequest.withOutput(commandedOutput));
    }

    /** Stop the spindexer. */
    public void stop() {
        commandedOutput = 0;
        spindexerMotor.setControl(dutyCycleRequest.withOutput(0));
    }

    // ==================== COMMAND FACTORIES ====================

    /**
     * Run spindexer at intake speed while held.
     * Use in parallel with intake command.
     */
    public Command intakeCommand() {
        return Commands.startEnd(
                this::runIntakeSpeed,
                this::stop,
                this
        ).withName("Spindexer Intake");
    }

    /**
     * Reverse spindexer while held (unjam).
     */
    public Command reverseCommand() {
        return Commands.startEnd(
                this::reverse,
                this::stop,
                this
        ).withName("Spindexer Reverse");
    }
}
