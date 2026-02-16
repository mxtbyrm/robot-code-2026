package frc.robot;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for RobotState logic.
 *
 * <p>Tests brownout prediction, voltage compensation factor, and
 * loop overrun counting without hardware or WPILib HAL.
 *
 * <p>The logic is extracted and replicated to avoid HAL/RobotController
 * dependencies.
 */
class RobotStateTest {

    // ==================== CONSTANTS (replicated from RobotState) ====================
    private static final double kBrownoutThresholdVolts = 7.0;
    private static final double kCriticalVoltageThreshold = 6.5;
    private static final double kNominalVoltage = 12.0;
    private static final int kVoltageHistorySize = 25;
    private static final double kBrownoutPredictionRateVPS = -4.0;

    // ==================== VOLTAGE COMPENSATION FACTOR ====================

    /**
     * Replicates RobotState.getVoltageCompensationFactor() logic.
     */
    private double computeVoltageCompensationFactor(double voltage) {
        if (voltage >= kNominalVoltage) return 1.0;
        if (voltage <= kCriticalVoltageThreshold) return 0.0;
        return (voltage - kCriticalVoltageThreshold) / (kNominalVoltage - kCriticalVoltageThreshold);
    }

    @Test
    void voltageCompensation_atNominal_returnsOne() {
        assertEquals(1.0, computeVoltageCompensationFactor(12.0), 0.001);
    }

    @Test
    void voltageCompensation_aboveNominal_returnsOne() {
        assertEquals(1.0, computeVoltageCompensationFactor(13.0), 0.001,
                "Above nominal should clamp to 1.0");
    }

    @Test
    void voltageCompensation_atCritical_returnsZero() {
        assertEquals(0.0, computeVoltageCompensationFactor(6.5), 0.001);
    }

    @Test
    void voltageCompensation_belowCritical_returnsZero() {
        assertEquals(0.0, computeVoltageCompensationFactor(5.0), 0.001,
                "Below critical should clamp to 0.0");
    }

    @Test
    void voltageCompensation_midRange_isLinear() {
        double mid = (kNominalVoltage + kCriticalVoltageThreshold) / 2.0;
        assertEquals(0.5, computeVoltageCompensationFactor(mid), 0.01,
                "Midpoint voltage should give ~0.5 factor");
    }

    @Test
    void voltageCompensation_isMonotonic() {
        double prev = computeVoltageCompensationFactor(6.5);
        for (double v = 7.0; v <= 12.0; v += 0.5) {
            double curr = computeVoltageCompensationFactor(v);
            assertTrue(curr >= prev,
                    "Factor should be monotonically increasing with voltage");
            prev = curr;
        }
    }

    @Test
    void voltageCompensation_at7V_isCorrect() {
        double expected = (7.0 - 6.5) / (12.0 - 6.5);
        assertEquals(expected, computeVoltageCompensationFactor(7.0), 0.001);
    }

    // ==================== BROWNOUT PREDICTION ====================

    /**
     * Replicates brownout prediction logic from RobotState.periodic().
     */
    private boolean predictBrownout(double currentVoltage, double oldestVoltage, double windowSeconds) {
        double dropRate = (currentVoltage - oldestVoltage) / windowSeconds;
        return currentVoltage < (kBrownoutThresholdVolts + 1.5)
                && dropRate < kBrownoutPredictionRateVPS;
    }

    @Test
    void brownoutPrediction_rapidDrop_predicts() {
        // 12V → 7.5V over 0.5s window = -9 V/s
        boolean predicted = predictBrownout(7.5, 12.0, 0.5);
        assertTrue(predicted, "Rapid voltage drop should predict brownout");
    }

    @Test
    void brownoutPrediction_stableVoltage_noPrediction() {
        boolean predicted = predictBrownout(12.0, 12.0, 0.5);
        assertFalse(predicted, "Stable voltage should not predict brownout");
    }

    @Test
    void brownoutPrediction_slowDrop_noPrediction() {
        // 12V → 11.5V over 0.5s = -1 V/s (below threshold of -4 V/s)
        boolean predicted = predictBrownout(11.5, 12.0, 0.5);
        assertFalse(predicted, "Slow voltage drop should not predict brownout");
    }

    @Test
    void brownoutPrediction_rapidDropButHighVoltage_noPrediction() {
        // Rapid drop but voltage is still well above threshold + 1.5
        boolean predicted = predictBrownout(11.0, 14.0, 0.5);
        assertFalse(predicted,
                "Rapid drop at high voltage should not predict brownout (still above 8.5V)");
    }

    @Test
    void brownoutPrediction_voltageRising_noPrediction() {
        boolean predicted = predictBrownout(8.0, 7.0, 0.5);
        assertFalse(predicted,
                "Rising voltage (positive rate) should not predict brownout");
    }

    @Test
    void brownoutPrediction_atThresholdBoundary() {
        // Current at exactly threshold + 1.5 = 8.5V
        // Drop: 8.5 - 12.0 over 0.5s = -7 V/s
        boolean predicted = predictBrownout(8.5, 12.0, 0.5);
        // 8.5 < 8.5 is false, so should NOT predict
        assertFalse(predicted,
                "At exactly threshold+1.5, condition is strict <, so no prediction");
    }

    @Test
    void brownoutPrediction_justBelowBoundary() {
        // Current at 8.49V with rapid drop
        boolean predicted = predictBrownout(8.49, 12.0, 0.5);
        assertTrue(predicted,
                "Just below threshold+1.5 with rapid drop should predict");
    }

    // ==================== BROWNOUT PREDICTION SLIDING WINDOW ====================

    @Test
    void voltageHistory_windowSize_isReasonable() {
        assertTrue(kVoltageHistorySize >= 10,
                "Window should be at least 10 samples for stable trend");
        assertTrue(kVoltageHistorySize <= 100,
                "Window should be at most 100 samples for responsiveness");
    }

    @Test
    void voltageHistory_windowDuration() {
        double windowSeconds = kVoltageHistorySize * 0.02; // 20ms loop period
        assertEquals(0.5, windowSeconds, 0.001,
                "25 samples at 20ms = 0.5 second window");
    }

    // ==================== LOOP OVERRUN COUNTING ====================

    /**
     * Replicates loop overrun detection logic.
     */
    private boolean isLoopOverrun(double loopTimeMs) {
        return loopTimeMs > 25.0; // 20ms nominal + 5ms margin
    }

    @Test
    void loopOverrun_normalLoop_notOverrun() {
        assertFalse(isLoopOverrun(20.0), "20ms loop should not be overrun");
    }

    @Test
    void loopOverrun_slightlyLong_notOverrun() {
        assertFalse(isLoopOverrun(24.9), "24.9ms loop should not be overrun (within margin)");
    }

    @Test
    void loopOverrun_overThreshold_isOverrun() {
        assertTrue(isLoopOverrun(26.0), "26ms loop should be overrun");
    }

    @Test
    void loopOverrun_veryLong_isOverrun() {
        assertTrue(isLoopOverrun(100.0), "100ms loop should definitely be overrun");
    }

    @Test
    void loopOverrun_counting() {
        int count = 0;
        double[] loopTimes = { 20.0, 22.0, 30.0, 15.0, 40.0, 20.0, 26.0 };
        for (double lt : loopTimes) {
            if (isLoopOverrun(lt)) count++;
        }
        assertEquals(3, count, "Should count 3 overruns (30ms, 40ms, 26ms)");
    }

    // ==================== HEALTH FLAGS ====================

    /**
     * Replicates RobotState.isAllHealthy() logic.
     */
    private boolean computeAllHealthy(boolean swerve, boolean shooter, boolean intake,
                                       boolean feeder, boolean spindexer, boolean vision) {
        return swerve && shooter && intake && feeder && spindexer && vision;
    }

    @Test
    void allHealthy_allTrue_returnsTrue() {
        assertTrue(computeAllHealthy(true, true, true, true, true, true));
    }

    @Test
    void allHealthy_oneUnhealthy_returnsFalse() {
        assertFalse(computeAllHealthy(false, true, true, true, true, true));
        assertFalse(computeAllHealthy(true, false, true, true, true, true));
        assertFalse(computeAllHealthy(true, true, false, true, true, true));
        assertFalse(computeAllHealthy(true, true, true, false, true, true));
        assertFalse(computeAllHealthy(true, true, true, true, false, true));
        assertFalse(computeAllHealthy(true, true, true, true, true, false));
    }

    @Test
    void allHealthy_allUnhealthy_returnsFalse() {
        assertFalse(computeAllHealthy(false, false, false, false, false, false));
    }

    // ==================== CRITICAL BATTERY ====================

    @Test
    void criticalBattery_atThreshold() {
        assertFalse(6.5 < kCriticalVoltageThreshold,
                "At exactly critical threshold, should not be critical (strict <)");
    }

    @Test
    void criticalBattery_belowThreshold() {
        assertTrue(6.0 < kCriticalVoltageThreshold,
                "Below critical threshold should be critical");
    }

    @Test
    void criticalBattery_aboveThreshold() {
        assertFalse(7.0 < kCriticalVoltageThreshold,
                "Above critical threshold should not be critical");
    }

    // ==================== LOW BATTERY ====================

    @Test
    void lowBattery_atThreshold() {
        assertFalse(7.0 < kBrownoutThresholdVolts,
                "At exactly brownout threshold, should not be low");
    }

    @Test
    void lowBattery_belowThreshold() {
        assertTrue(6.9 < kBrownoutThresholdVolts,
                "Below brownout threshold should be low battery");
    }

    // ==================== FAULT COUNTING ====================

    @Test
    void faultCount_incrementsOnTransitionToUnhealthy() {
        int totalFaultCount = 0;
        boolean wasHealthy = true;

        // First transition to unhealthy
        boolean nowHealthy = false;
        if (wasHealthy && !nowHealthy) totalFaultCount++;
        assertEquals(1, totalFaultCount);

        // Stay unhealthy — should NOT increment again
        wasHealthy = nowHealthy;
        nowHealthy = false;
        if (wasHealthy && !nowHealthy) totalFaultCount++;
        assertEquals(1, totalFaultCount, "Should not increment when staying unhealthy");

        // Recover
        wasHealthy = nowHealthy;
        nowHealthy = true;

        // Second fault
        wasHealthy = nowHealthy;
        nowHealthy = false;
        if (wasHealthy && !nowHealthy) totalFaultCount++;
        assertEquals(2, totalFaultCount, "Should increment on new fault after recovery");
    }
}
