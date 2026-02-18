package frc.robot;

import frc.robot.constants.BallDetectionConstants;

import org.littletonrobotics.junction.Logger;

/**
 * Sensorless ball presence estimator using motor current analysis.
 *
 * <h2>Principle:</h2>
 * When a motor is spinning with no load (empty hopper), it draws a baseline
 * current. When balls are present, friction and mass increase the load →
 * current rises above baseline. The delta tells us if balls are present.
 *
 * <h2>Three current sources, three signals:</h2>
 * <ol>
 *   <li><b>Spindexer current</b> — balls in hopper add friction → current rises.
 *       High delta = hopper loaded, low delta = hopper empty.</li>
 *   <li><b>Feeder current</b> — current spike when a ball is being pushed
 *       from hopper into the flywheel contact zone.</li>
 *   <li><b>Beam break</b> (between feeder and shooter) — definitive ball exit
 *       confirmation. Already used for shot counting in Superstructure.</li>
 * </ol>
 *
 * <h2>Ball count estimation:</h2>
 * <pre>
 *   estimated_balls = preloaded_balls + intake_events − beam_break_exits
 * </pre>
 * Since we can't count individual balls entering the hopper precisely,
 * we estimate hopper fullness from spindexer current magnitude instead.
 *
 * <h2>Calibration:</h2>
 * Baseline current is recorded during the pre-match system health check
 * when the spindexer/feeder run empty. This accounts for per-robot
 * variations in friction, lubrication, and belt tension.
 *
 * <h2>Usage:</h2>
 * <ul>
 *   <li>{@code isHopperEmpty()} — Superstructure can auto-stop shooting</li>
 *   <li>{@code isHopperLoaded()} — LEDs / dashboard indicator for driver</li>
 *   <li>{@code isBallAtFeeder()} — feeder detects a ball arriving</li>
 *   <li>{@code getHopperLoadLevel()} — 0.0 (empty) to 1.0 (full) estimate</li>
 * </ul>
 */
public class BallPresenceEstimator {

    // ==================== SINGLETON ====================
    private static BallPresenceEstimator instance;

    public static BallPresenceEstimator getInstance() {
        if (instance == null) instance = new BallPresenceEstimator();
        return instance;
    }

    public static void resetInstance() {
        instance = null;
    }

    // ==================== CALIBRATION BASELINES ====================
    // Set during system health check when mechanisms run empty.
    // Adaptive: slowly drift toward smoothed current when delta is very low
    // (motor spinning empty), which compensates for thermal resistance changes.
    private double spindexerBaselineCurrent = BallDetectionConstants.kDefaultSpindexerBaselineCurrent;
    private double feederBaselineCurrent = BallDetectionConstants.kDefaultFeederBaselineCurrent;
    private boolean calibrated = false;
    // How fast baseline adapts: fraction per cycle. 0.002 at 50Hz ≈ 1s time constant.
    private static final double kBaselineAdaptRate = 0.002;

    // ==================== RUNNING AVERAGES ====================
    // Smoothed current readings to filter out electrical noise.
    private final double[] spindexerCurrentHistory = new double[BallDetectionConstants.kCurrentHistorySize];
    private final double[] feederCurrentHistory = new double[BallDetectionConstants.kCurrentHistorySize];
    private int historyIndex = 0;
    private boolean historyFilled = false;

    // ==================== STATE ====================
    private double smoothedSpindexerCurrent = 0.0;
    private double smoothedFeederCurrent = 0.0;
    private double spindexerDelta = 0.0;
    private double feederDelta = 0.0;
    private boolean hopperLoaded = false;
    private boolean hopperEmpty = true;
    private boolean ballAtFeeder = false;
    private double hopperLoadLevel = 0.0;

    private BallPresenceEstimator() {}

    // ==================== CALIBRATION ====================

    /**
     * Record baseline currents from empty mechanisms during health check.
     * Call this when spindexer/feeder are running with NO balls loaded.
     *
     * @param spindexerEmptyCurrent average spindexer current when empty
     * @param feederEmptyCurrent    average feeder current when empty
     */
    public void calibrate(double spindexerEmptyCurrent, double feederEmptyCurrent) {
        this.spindexerBaselineCurrent = spindexerEmptyCurrent;
        this.feederBaselineCurrent = feederEmptyCurrent;
        this.calibrated = true;
    }

    /** @return true if baselines have been calibrated from health check */
    public boolean isCalibrated() {
        return calibrated;
    }

    // ==================== PERIODIC UPDATE ====================

    /**
     * Update with latest current readings. Call every robot cycle (20ms)
     * from Superstructure.periodic().
     *
     * @param spindexerCurrent instantaneous spindexer stator current (amps)
     * @param feederCurrent    instantaneous feeder stator current (amps)
     * @param spindexerRunning true if spindexer is actively commanded (ignore current when idle)
     * @param feederRunning    true if feeder is actively running
     */
    public void update(double spindexerCurrent, double feederCurrent,
                       boolean spindexerRunning, boolean feederRunning) {

        // ---- Voltage compensation ----
        // At lower bus voltage, motors draw more current for the same mechanical load.
        // Normalize readings to 12V nominal so thresholds remain valid during voltage sag.
        // Scale factor: (nominalVoltage / actualVoltage) inverted because current goes UP
        // as voltage goes DOWN. We scale current DOWN to what it would be at nominal voltage.
        double busVoltage = RobotState.getInstance().getBatteryVoltage();
        if (busVoltage > 8.0 && busVoltage < 12.5) {
            double voltageScale = busVoltage / BallDetectionConstants.kNominalCalibrationVoltage;
            spindexerCurrent *= voltageScale;
            feederCurrent *= voltageScale;
        }

        // ---- Sliding window average ----
        spindexerCurrentHistory[historyIndex] = spindexerRunning ? spindexerCurrent : 0;
        feederCurrentHistory[historyIndex] = feederRunning ? feederCurrent : 0;
        historyIndex = (historyIndex + 1) % BallDetectionConstants.kCurrentHistorySize;
        if (historyIndex == 0) historyFilled = true;

        int count = historyFilled ? BallDetectionConstants.kCurrentHistorySize : historyIndex;
        if (count > 0) {
            double spSum = 0, fSum = 0;
            for (int i = 0; i < count; i++) {
                spSum += spindexerCurrentHistory[i];
                fSum += feederCurrentHistory[i];
            }
            smoothedSpindexerCurrent = spSum / count;
            smoothedFeederCurrent = fSum / count;
        }

        // ---- Compute deltas from baseline ----
        if (spindexerRunning) {
            spindexerDelta = smoothedSpindexerCurrent - spindexerBaselineCurrent;
        }
        if (feederRunning) {
            feederDelta = smoothedFeederCurrent - feederBaselineCurrent;
        }

        // ---- Adaptive baseline drift compensation ----
        // When motors are running and delta is very low (well below "loaded" threshold),
        // slowly drift baseline toward the current reading. This compensates for thermal
        // resistance changes during a match without needing explicit re-calibration.
        // Only adapt when historyFilled to ensure we have stable smoothed values.
        if (historyFilled) {
            if (spindexerRunning && spindexerDelta < BallDetectionConstants.kSpindexerEmptyThreshold) {
                spindexerBaselineCurrent += (smoothedSpindexerCurrent - spindexerBaselineCurrent) * kBaselineAdaptRate;
            }
            if (feederRunning && feederDelta < BallDetectionConstants.kFeederBallPresentThreshold * 0.3) {
                feederBaselineCurrent += (smoothedFeederCurrent - feederBaselineCurrent) * kBaselineAdaptRate;
            }
        }

        // ---- Determine hopper state from spindexer delta ----
        if (spindexerRunning && historyFilled) {
            hopperLoaded = spindexerDelta > BallDetectionConstants.kSpindexerLoadedThreshold;
            hopperEmpty = spindexerDelta < BallDetectionConstants.kSpindexerEmptyThreshold;

            // Normalize to 0..1 load level
            double maxDelta = BallDetectionConstants.kSpindexerFullLoadDelta;
            hopperLoadLevel = Math.min(Math.max(spindexerDelta / maxDelta, 0.0), 1.0);
        }
        // When spindexer is idle, retain last known state — don't reset to unknown.

        // ---- Detect ball at feeder entrance ----
        if (feederRunning) {
            ballAtFeeder = feederDelta > BallDetectionConstants.kFeederBallPresentThreshold;
        } else {
            ballAtFeeder = false;
        }

        // ---- Telemetry (AdvantageKit only) ----
        Logger.recordOutput("BallEst/SpindexerDelta", spindexerDelta);
        Logger.recordOutput("BallEst/FeederDelta", feederDelta);
        Logger.recordOutput("BallEst/HopperLoaded", hopperLoaded);
        Logger.recordOutput("BallEst/HopperEmpty", hopperEmpty);
        Logger.recordOutput("BallEst/BallAtFeeder", ballAtFeeder);
        Logger.recordOutput("BallEst/HopperLoadLevel", hopperLoadLevel);
        Logger.recordOutput("BallEst/Calibrated", calibrated);
        Logger.recordOutput("BallEst/SpindexerBaseline", spindexerBaselineCurrent);
        Logger.recordOutput("BallEst/FeederBaseline", feederBaselineCurrent);
    }

    // ==================== GETTERS ====================

    /**
     * @return true if hopper appears empty (spindexer current near baseline).
     *         Useful for auto-stopping shooting or signaling driver to intake.
     */
    public boolean isHopperEmpty() {
        return hopperEmpty;
    }

    /**
     * @return true if hopper has balls (spindexer current significantly above baseline).
     */
    public boolean isHopperLoaded() {
        return hopperLoaded;
    }

    /**
     * @return 0.0 (empty) to 1.0 (full) estimate of hopper fullness.
     *         Based on current delta magnitude.
     */
    public double getHopperLoadLevel() {
        return hopperLoadLevel;
    }

    /**
     * @return true if feeder current suggests a ball is present at the feeder entrance.
     */
    public boolean isBallAtFeeder() {
        return ballAtFeeder;
    }

    /** @return raw spindexer current delta from baseline (amps) */
    public double getSpindexerDelta() {
        return spindexerDelta;
    }

    /** @return raw feeder current delta from baseline (amps) */
    public double getFeederDelta() {
        return feederDelta;
    }
}
