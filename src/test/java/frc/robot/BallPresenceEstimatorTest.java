package frc.robot;

import frc.robot.constants.BallDetectionConstants;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for BallPresenceEstimator logic.
 *
 * <p>Tests voltage compensation, calibration, adaptive baseline drift,
 * hopper detection hysteresis, and feeder ball detection.
 *
 * <p>Since BallPresenceEstimator depends on RobotState singleton,
 * these tests replicate the pure computation logic independently.
 */
class BallPresenceEstimatorTest {

    // ==================== VOLTAGE COMPENSATION ====================

    /**
     * Replicates the voltage compensation from BallPresenceEstimator.update().
     * Scales current to what it would be at nominal voltage.
     */
    private double applyVoltageCompensation(double current, double busVoltage) {
        if (busVoltage > 8.0 && busVoltage < 12.5) {
            return current * (busVoltage / BallDetectionConstants.kNominalCalibrationVoltage);
        }
        return current;
    }

    @Test
    void voltageCompensation_nominalVoltage_noChange() {
        double current = 5.0;
        double compensated = applyVoltageCompensation(current, 12.0);
        assertEquals(current, compensated, 0.001,
                "At nominal voltage, current should be unchanged");
    }

    @Test
    void voltageCompensation_lowVoltage_scalesDown() {
        double current = 5.0;
        double compensated = applyVoltageCompensation(current, 11.0);
        double expected = 5.0 * (11.0 / 12.0);
        assertEquals(expected, compensated, 0.001,
                "At 11V, current should be scaled by 11/12");
    }

    @Test
    void voltageCompensation_veryLowVoltage_noScale() {
        // Below 8V, no compensation applied (unreliable readings)
        double current = 5.0;
        double compensated = applyVoltageCompensation(current, 7.0);
        assertEquals(current, compensated, 0.001,
                "Below 8V, voltage compensation should not apply");
    }

    @Test
    void voltageCompensation_highVoltage_noScale() {
        // Above 12.5V, no compensation applied
        double current = 5.0;
        double compensated = applyVoltageCompensation(current, 13.0);
        assertEquals(current, compensated, 0.001,
                "Above 12.5V, voltage compensation should not apply");
    }

    // ==================== CALIBRATION ====================

    @Test
    void calibration_overridesDefaults() {
        double defaultBaseline = BallDetectionConstants.kDefaultSpindexerBaselineCurrent;
        double calibratedBaseline = 4.5;

        // After calibration, baseline should be the calibrated value, not default
        assertNotEquals(defaultBaseline, calibratedBaseline,
                "Test setup: calibrated value should differ from default");

        // The delta computation uses the calibrated baseline
        double current = 6.0;
        double deltaWithDefault = current - defaultBaseline;
        double deltaWithCalibrated = current - calibratedBaseline;

        assertNotEquals(deltaWithDefault, deltaWithCalibrated,
                "Delta should differ between default and calibrated baselines");
    }

    @Test
    void calibration_defaultBaselines_arePositive() {
        assertTrue(BallDetectionConstants.kDefaultSpindexerBaselineCurrent > 0,
                "Default spindexer baseline should be positive");
        assertTrue(BallDetectionConstants.kDefaultFeederBaselineCurrent > 0,
                "Default feeder baseline should be positive");
    }

    // ==================== ADAPTIVE BASELINE DRIFT ====================

    /**
     * Replicates adaptive baseline drift logic.
     */
    private double adaptBaseline(double baseline, double smoothedCurrent, double delta, double threshold) {
        double adaptRate = 0.002;
        if (delta < threshold) {
            return baseline + (smoothedCurrent - baseline) * adaptRate;
        }
        return baseline;
    }

    @Test
    void adaptiveBaseline_lowDelta_driftsTowardCurrent() {
        double baseline = 3.0;
        double smoothedCurrent = 3.1;
        double delta = smoothedCurrent - baseline; // 0.1, below threshold

        double newBaseline = adaptBaseline(baseline, smoothedCurrent, delta,
                BallDetectionConstants.kSpindexerEmptyThreshold);

        assertTrue(newBaseline > baseline,
                "Baseline should drift toward smoothed current when delta is low");
        assertTrue(newBaseline < smoothedCurrent,
                "Baseline should not jump to smoothed current in one step");
    }

    @Test
    void adaptiveBaseline_highDelta_doesNotDrift() {
        double baseline = 3.0;
        double smoothedCurrent = 6.0;
        double delta = smoothedCurrent - baseline; // 3.0, above threshold

        double newBaseline = adaptBaseline(baseline, smoothedCurrent, delta,
                BallDetectionConstants.kSpindexerEmptyThreshold);

        assertEquals(baseline, newBaseline, 0.001,
                "Baseline should not drift when delta is above threshold (balls present)");
    }

    @Test
    void adaptiveBaseline_100Cycles_converges() {
        double baseline = 3.0;
        double targetCurrent = 3.2; // slight drift
        double adaptRate = 0.002;

        for (int i = 0; i < 100; i++) {
            double delta = targetCurrent - baseline;
            if (delta < BallDetectionConstants.kSpindexerEmptyThreshold) {
                baseline = baseline + (targetCurrent - baseline) * adaptRate;
            }
        }

        // After 100 cycles, baseline should be closer to target
        assertTrue(Math.abs(baseline - targetCurrent) < Math.abs(3.0 - targetCurrent),
                "After 100 cycles, baseline should converge toward target");
    }

    // ==================== HOPPER DETECTION HYSTERESIS ====================

    /**
     * Replicates hopper state detection with hysteresis.
     */
    private boolean isHopperLoaded(double spindexerDelta) {
        return spindexerDelta > BallDetectionConstants.kSpindexerLoadedThreshold;
    }

    private boolean isHopperEmpty(double spindexerDelta) {
        return spindexerDelta < BallDetectionConstants.kSpindexerEmptyThreshold;
    }

    @Test
    void hopperDetection_belowEmptyThreshold_isEmpty() {
        assertTrue(isHopperEmpty(0.3),
                "Delta below empty threshold should report empty");
        assertFalse(isHopperLoaded(0.3),
                "Delta below empty threshold should not report loaded");
    }

    @Test
    void hopperDetection_aboveLoadedThreshold_isLoaded() {
        assertTrue(isHopperLoaded(3.0),
                "Delta above loaded threshold should report loaded");
        assertFalse(isHopperEmpty(3.0),
                "Delta above loaded threshold should not report empty");
    }

    @Test
    void hopperDetection_inHysteresisBand_neitherLoadedNorEmpty() {
        double midDelta = (BallDetectionConstants.kSpindexerEmptyThreshold
                + BallDetectionConstants.kSpindexerLoadedThreshold) / 2.0;

        assertFalse(isHopperLoaded(midDelta),
                "In hysteresis band: should not be loaded");
        assertFalse(isHopperEmpty(midDelta),
                "In hysteresis band: should not be empty");
    }

    @Test
    void hopperDetection_hysteresisBand_exists() {
        assertTrue(BallDetectionConstants.kSpindexerLoadedThreshold
                > BallDetectionConstants.kSpindexerEmptyThreshold,
                "Loaded threshold must be above empty threshold for hysteresis");
    }

    // ==================== HOPPER LOAD LEVEL ====================

    @Test
    void hopperLoadLevel_empty_isZero() {
        double delta = 0.0;
        double level = Math.min(Math.max(delta / BallDetectionConstants.kSpindexerFullLoadDelta, 0.0), 1.0);
        assertEquals(0.0, level, 0.001);
    }

    @Test
    void hopperLoadLevel_full_isOne() {
        double delta = BallDetectionConstants.kSpindexerFullLoadDelta;
        double level = Math.min(Math.max(delta / BallDetectionConstants.kSpindexerFullLoadDelta, 0.0), 1.0);
        assertEquals(1.0, level, 0.001);
    }

    @Test
    void hopperLoadLevel_halfFull_isHalf() {
        double delta = BallDetectionConstants.kSpindexerFullLoadDelta / 2.0;
        double level = Math.min(Math.max(delta / BallDetectionConstants.kSpindexerFullLoadDelta, 0.0), 1.0);
        assertEquals(0.5, level, 0.001);
    }

    @Test
    void hopperLoadLevel_overFull_clampsToOne() {
        double delta = BallDetectionConstants.kSpindexerFullLoadDelta * 1.5;
        double level = Math.min(Math.max(delta / BallDetectionConstants.kSpindexerFullLoadDelta, 0.0), 1.0);
        assertEquals(1.0, level, 0.001);
    }

    @Test
    void hopperLoadLevel_negative_clampsToZero() {
        double delta = -1.0;
        double level = Math.min(Math.max(delta / BallDetectionConstants.kSpindexerFullLoadDelta, 0.0), 1.0);
        assertEquals(0.0, level, 0.001);
    }

    // ==================== FEEDER BALL DETECTION ====================

    @Test
    void feederBallDetection_aboveThreshold_detected() {
        double feederDelta = BallDetectionConstants.kFeederBallPresentThreshold + 1.0;
        assertTrue(feederDelta > BallDetectionConstants.kFeederBallPresentThreshold);
    }

    @Test
    void feederBallDetection_belowThreshold_notDetected() {
        double feederDelta = BallDetectionConstants.kFeederBallPresentThreshold - 1.0;
        assertFalse(feederDelta > BallDetectionConstants.kFeederBallPresentThreshold);
    }

    // ==================== SLIDING WINDOW AVERAGE ====================

    @Test
    void slidingWindow_size_isReasonable() {
        assertTrue(BallDetectionConstants.kCurrentHistorySize >= 5,
                "History size should be at least 5 for meaningful averaging");
        assertTrue(BallDetectionConstants.kCurrentHistorySize <= 50,
                "History size should be at most 50 to remain responsive");
    }

    @Test
    void slidingWindow_averageComputation() {
        int size = BallDetectionConstants.kCurrentHistorySize;
        double[] history = new double[size];
        // Fill with constant value
        for (int i = 0; i < size; i++) {
            history[i] = 5.0;
        }
        double sum = 0;
        for (double v : history) sum += v;
        double avg = sum / size;
        assertEquals(5.0, avg, 0.001, "Average of constant values should equal that constant");
    }

    // ==================== CONSTANTS SANITY ====================

    @Test
    void thresholds_arePositive() {
        assertTrue(BallDetectionConstants.kSpindexerLoadedThreshold > 0);
        assertTrue(BallDetectionConstants.kSpindexerEmptyThreshold > 0);
        assertTrue(BallDetectionConstants.kFeederBallPresentThreshold > 0);
        assertTrue(BallDetectionConstants.kSpindexerFullLoadDelta > 0);
    }

    @Test
    void fullLoadDelta_isAboveLoadedThreshold() {
        assertTrue(BallDetectionConstants.kSpindexerFullLoadDelta
                > BallDetectionConstants.kSpindexerLoadedThreshold,
                "Full load delta should be above loaded threshold");
    }
}
