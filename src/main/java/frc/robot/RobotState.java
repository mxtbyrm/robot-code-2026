package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

/**
 * Centralized robot health monitor for Einstein-grade diagnostics.
 *
 * <p>Runs every cycle and publishes:
 * <ul>
 *   <li>Battery voltage + brownout state</li>
 *   <li>CAN bus utilization</li>
 *   <li>Loop overrun detection (>20ms cycles)</li>
 *   <li>Match time + FMS state</li>
 *   <li>Per-subsystem health flags (set by each subsystem)</li>
 * </ul>
 *
 * <p>All Einstein-level teams have a system like this — it catches
 * hardware faults mid-match before they become catastrophic.
 */
public class RobotState extends SubsystemBase {

    // ==================== SINGLETON ====================
    private static RobotState instance;

    public static RobotState getInstance() {
        if (instance == null) instance = new RobotState();
        return instance;
    }

    /**
     * Inject a custom instance (for unit testing with mocks).
     * Call before any getInstance() usage in tests.
     */
    public static void setInstance(RobotState customInstance) {
        instance = customInstance;
    }

    /** Reset singleton (call in test teardown). */
    public static void resetInstance() {
        instance = null;
    }

    // ==================== BROWNOUT PROTECTION ====================
    /** Below this voltage, reduce motor output to prevent brownout. */
    public static final double kBrownoutThresholdVolts = 7.0;
    /** Below this voltage, disable non-critical subsystems. */
    public static final double kCriticalVoltageThreshold = 6.5;
    /** Voltage at which we consider battery healthy. */
    public static final double kNominalVoltage = 12.0;

    // ==================== LOOP TIMING ====================
    private double lastLoopTimestamp = 0;
    private double worstLoopTimeMs = 0;
    private int loopOverrunCount = 0;

    // ==================== SUBSYSTEM HEALTH FLAGS ====================
    private boolean swerveHealthy = true;
    private boolean shooterHealthy = true;
    private boolean intakeHealthy = true;
    private boolean feederHealthy = true;
    private boolean spindexerHealthy = true;
    private boolean visionHealthy = true;

    // ==================== FAULT COUNTERS ====================
    private int canErrorCount = 0;
    private int totalFaultCount = 0;

    // ==================== BROWNOUT PREDICTION ====================
    /**
     * Sliding window of recent voltage readings for trend analysis.
     * Used to predict brownout BEFORE it happens by detecting rapid voltage drop.
     */
    private static final int kVoltageHistorySize = 25; // ~0.5 seconds at 50Hz
    private final double[] voltageHistory = new double[kVoltageHistorySize];
    private int voltageHistoryIndex = 0;
    private boolean voltageHistoryFilled = false;
    /** True when voltage is dropping rapidly — brownout is imminent. */
    private boolean brownoutPredicted = false;
    /** Rate of voltage change in V/s (negative = dropping). */
    private double voltageDropRateVPS = 0.0;
    /** Threshold: if voltage is dropping faster than this (V/s), predict brownout. */
    private static final double kBrownoutPredictionRateVPS = -4.0; // -4V/s = aggressive draw

    private RobotState() {
        lastLoopTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void periodic() {
        // ---- Loop timing ----
        double now = Timer.getFPGATimestamp();
        double loopTimeMs = (now - lastLoopTimestamp) * 1000.0;
        lastLoopTimestamp = now;

        if (loopTimeMs > 25.0) { // 20ms nominal + 5ms margin
            loopOverrunCount++;
        }
        if (loopTimeMs > worstLoopTimeMs) {
            worstLoopTimeMs = loopTimeMs;
        }

        // ---- Battery / power ----
        double voltage = RobotController.getBatteryVoltage();
        boolean brownout = RobotController.isBrownedOut();
        boolean lowBattery = voltage < kBrownoutThresholdVolts;
        boolean criticalBattery = voltage < kCriticalVoltageThreshold;

        // ---- Brownout prediction via voltage trend ----
        voltageHistory[voltageHistoryIndex] = voltage;
        voltageHistoryIndex = (voltageHistoryIndex + 1) % kVoltageHistorySize;
        if (voltageHistoryIndex == 0) voltageHistoryFilled = true;

        if (voltageHistoryFilled) {
            // Compare newest vs oldest reading in the window
            int oldestIdx = voltageHistoryIndex; // wraps to oldest after increment
            double oldestVoltage = voltageHistory[oldestIdx];
            double windowSeconds = kVoltageHistorySize * 0.02; // 20ms loop period
            voltageDropRateVPS = (voltage - oldestVoltage) / windowSeconds;
            brownoutPredicted = voltage < (kBrownoutThresholdVolts + 1.5)
                    && voltageDropRateVPS < kBrownoutPredictionRateVPS;
        }

        // ---- CAN bus ----
        // Cache the CAN status — getCANStatus() is a JNI call, avoid calling it 3x per cycle.
        var canStatus = RobotController.getCANStatus();
        int canTxErrors = canStatus.transmitErrorCount;
        int canRxErrors = canStatus.receiveErrorCount;
        double canUtilization = canStatus.percentBusUtilization;
        canErrorCount = canTxErrors + canRxErrors;

        // CAN bus high-utilization warning
        if (canUtilization > 0.70) {
            DriverStation.reportWarning(
                    "[RobotState] CAN utilization HIGH: " + String.format("%.0f%%", canUtilization * 100), false);
        }
        if (canErrorCount > 0 && loopTimeMs > 0) {
            DriverStation.reportWarning(
                    "[RobotState] CAN errors detected: TX=" + canTxErrors + " RX=" + canRxErrors, false);
        }

        // ---- Overall health ----
        boolean allHealthy = swerveHealthy && shooterHealthy && intakeHealthy
                && feederHealthy && spindexerHealthy && visionHealthy
                && !brownout && canErrorCount == 0;

        // ---- Telemetry (AdvantageKit only) ----
        Logger.recordOutput("RobotState/BatteryVoltage", voltage);
        Logger.recordOutput("RobotState/Brownout", brownout);
        Logger.recordOutput("RobotState/LowBattery", lowBattery);
        Logger.recordOutput("RobotState/CriticalBattery", criticalBattery);
        Logger.recordOutput("RobotState/BrownoutPredicted", brownoutPredicted);
        Logger.recordOutput("RobotState/VoltageDropRateVPS", voltageDropRateVPS);
        Logger.recordOutput("RobotState/CANUtilization", canUtilization * 100.0);
        Logger.recordOutput("RobotState/CANErrors", canErrorCount);
        Logger.recordOutput("RobotState/LoopOverruns", loopOverrunCount);
        Logger.recordOutput("RobotState/WorstLoopMs", worstLoopTimeMs);
        Logger.recordOutput("RobotState/LoopTimeMs", loopTimeMs);
        Logger.recordOutput("RobotState/AllHealthy", allHealthy);
        Logger.recordOutput("RobotState/SwerveOK", swerveHealthy);
        Logger.recordOutput("RobotState/ShooterOK", shooterHealthy);
        Logger.recordOutput("RobotState/IntakeOK", intakeHealthy);
        Logger.recordOutput("RobotState/FeederOK", feederHealthy);
        Logger.recordOutput("RobotState/SpindexerOK", spindexerHealthy);
        Logger.recordOutput("RobotState/VisionOK", visionHealthy);
        Logger.recordOutput("RobotState/TotalFaults", totalFaultCount);

        if (DriverStation.isFMSAttached()) {
            Logger.recordOutput("RobotState/MatchTime", DriverStation.getMatchTime());
        }
    }

    // ==================== HEALTH SETTERS (called by each subsystem) ====================

    public void setSwerveHealthy(boolean healthy) {
        if (swerveHealthy && !healthy) totalFaultCount++;
        swerveHealthy = healthy;
    }

    public void setShooterHealthy(boolean healthy) {
        if (shooterHealthy && !healthy) totalFaultCount++;
        shooterHealthy = healthy;
    }

    public void setIntakeHealthy(boolean healthy) {
        if (intakeHealthy && !healthy) totalFaultCount++;
        intakeHealthy = healthy;
    }

    public void setFeederHealthy(boolean healthy) {
        if (feederHealthy && !healthy) totalFaultCount++;
        feederHealthy = healthy;
    }

    public void setSpindexerHealthy(boolean healthy) {
        if (spindexerHealthy && !healthy) totalFaultCount++;
        spindexerHealthy = healthy;
    }

    public void setVisionHealthy(boolean healthy) {
        if (visionHealthy && !healthy) totalFaultCount++;
        visionHealthy = healthy;
    }

    // ==================== QUERIES ====================

    /** @return current battery voltage */
    public double getBatteryVoltage() {
        return RobotController.getBatteryVoltage();
    }

    /** @return true if battery is critically low */
    public boolean isCriticalBattery() {
        return getBatteryVoltage() < kCriticalVoltageThreshold;
    }

    /** @return true if battery is in brownout territory */
    public boolean isLowBattery() {
        return getBatteryVoltage() < kBrownoutThresholdVolts;
    }

    /**
     * @return true if brownout is predicted based on voltage drop rate.
     *         When true, subsystems should proactively reduce power draw
     *         before the actual brownout occurs.
     */
    public boolean isBrownoutPredicted() {
        return brownoutPredicted;
    }

    /** @return voltage drop rate in V/s (negative = dropping). */
    public double getVoltageDropRate() {
        return voltageDropRateVPS;
    }

    /**
     * Returns a voltage compensation factor (0.0–1.0) to scale motor outputs.
     * At nominal voltage returns 1.0. At brownout threshold returns ~0.6.
     * Use this to gracefully degrade instead of hard-browning-out.
     */
    public double getVoltageCompensationFactor() {
        double v = getBatteryVoltage();
        if (v >= kNominalVoltage) return 1.0;
        if (v <= kCriticalVoltageThreshold) return 0.0;
        return (v - kCriticalVoltageThreshold) / (kNominalVoltage - kCriticalVoltageThreshold);
    }

    /** @return true if all subsystems report healthy */
    public boolean isAllHealthy() {
        return swerveHealthy && shooterHealthy && intakeHealthy
                && feederHealthy && spindexerHealthy && visionHealthy;
    }

    /** @return CAN bus utilization 0–1 */
    public double getCANUtilization() {
        return RobotController.getCANStatus().percentBusUtilization;
    }

    /** Reset loop timing stats (call at match start). */
    public void resetTimingStats() {
        worstLoopTimeMs = 0;
        loopOverrunCount = 0;
    }
}
