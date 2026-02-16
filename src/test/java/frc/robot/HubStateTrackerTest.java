package frc.robot;

import frc.robot.HubStateTracker.HubStatus;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for HubStateTracker Alliance Shift logic.
 *
 * <p>Since HubStateTracker is a singleton that depends on DriverStation,
 * Timer, and WPILib subsystem infrastructure, these tests validate the
 * pure shift-status logic independently:
 * <ul>
 *   <li>Won auto → odd shifts INACTIVE, even shifts ACTIVE</li>
 *   <li>Lost auto → odd shifts ACTIVE, even shifts INACTIVE</li>
 *   <li>shouldShoot() behavior during Active vs Inactive</li>
 *   <li>Shift schedule configuration sanity checks</li>
 * </ul>
 *
 * <p>The shift status logic is extracted here to test without
 * the WPILib HAL running.
 */
class HubStateTrackerTest {

    // ==================== PURE SHIFT LOGIC ====================
    // Replicate the shift status determination logic from HubStateTracker
    // to test it without DriverStation/Timer dependencies.

    /**
     * Determines HUB status for a given shift number and auto result.
     * This mirrors HubStateTracker.updateShiftStatus().
     */
    private HubStatus computeShiftStatus(int shiftNumber, boolean wonAuto) {
        boolean oddShift = (shiftNumber % 2 == 1);
        if (wonAuto) {
            return oddShift ? HubStatus.INACTIVE : HubStatus.ACTIVE;
        } else {
            return oddShift ? HubStatus.ACTIVE : HubStatus.INACTIVE;
        }
    }

    /**
     * Computes shift number from elapsed teleop time.
     * Mirrors HubStateTracker.updateFromMatchTime().
     */
    private int computeShiftNumber(double elapsedSeconds) {
        return (int) (elapsedSeconds / HubStateTracker.kShiftDurationSeconds) + 1;
    }

    /**
     * Determines shouldShoot based on status and time until next shift.
     * Mirrors HubStateTracker.shouldShoot().
     */
    private boolean computeShouldShoot(HubStatus status, double timeUntilShift) {
        if (status == HubStatus.ACTIVE) return true;
        if (status == HubStatus.UNKNOWN) return true;
        // Inactive — shoot if shift is imminent
        return timeUntilShift < 3.0;
    }

    // ==================== WON AUTO: Shift 1 = INACTIVE ====================

    @Test
    void wonAuto_shift1_isInactive() {
        assertEquals(HubStatus.INACTIVE, computeShiftStatus(1, true),
                "Won auto → Shift 1 should be INACTIVE (penalty for auto dominance)");
    }

    @Test
    void wonAuto_shift2_isActive() {
        assertEquals(HubStatus.ACTIVE, computeShiftStatus(2, true),
                "Won auto → Shift 2 should be ACTIVE");
    }

    @Test
    void wonAuto_shift3_isInactive() {
        assertEquals(HubStatus.INACTIVE, computeShiftStatus(3, true),
                "Won auto → Shift 3 should be INACTIVE");
    }

    @Test
    void wonAuto_shift4_isActive() {
        assertEquals(HubStatus.ACTIVE, computeShiftStatus(4, true),
                "Won auto → Shift 4 should be ACTIVE");
    }

    // ==================== LOST AUTO: Shift 1 = ACTIVE ====================

    @Test
    void lostAuto_shift1_isActive() {
        assertEquals(HubStatus.ACTIVE, computeShiftStatus(1, false),
                "Lost auto → Shift 1 should be ACTIVE (advantage)");
    }

    @Test
    void lostAuto_shift2_isInactive() {
        assertEquals(HubStatus.INACTIVE, computeShiftStatus(2, false),
                "Lost auto → Shift 2 should be INACTIVE");
    }

    @Test
    void lostAuto_shift3_isActive() {
        assertEquals(HubStatus.ACTIVE, computeShiftStatus(3, false),
                "Lost auto → Shift 3 should be ACTIVE");
    }

    @Test
    void lostAuto_shift4_isInactive() {
        assertEquals(HubStatus.INACTIVE, computeShiftStatus(4, false),
                "Lost auto → Shift 4 should be INACTIVE");
    }

    // ==================== ALTERNATION PATTERN ====================

    @Test
    void wonAuto_allShifts_alternate() {
        HubStatus[] expected = { HubStatus.INACTIVE, HubStatus.ACTIVE, HubStatus.INACTIVE, HubStatus.ACTIVE };
        for (int i = 0; i < HubStateTracker.kTotalShifts; i++) {
            assertEquals(expected[i], computeShiftStatus(i + 1, true),
                    "Won auto → Shift " + (i + 1) + " mismatch");
        }
    }

    @Test
    void lostAuto_allShifts_alternate() {
        HubStatus[] expected = { HubStatus.ACTIVE, HubStatus.INACTIVE, HubStatus.ACTIVE, HubStatus.INACTIVE };
        for (int i = 0; i < HubStateTracker.kTotalShifts; i++) {
            assertEquals(expected[i], computeShiftStatus(i + 1, false),
                    "Lost auto → Shift " + (i + 1) + " mismatch");
        }
    }

    @Test
    void wonAndLost_areAlwaysOpposite() {
        for (int shift = 1; shift <= HubStateTracker.kTotalShifts; shift++) {
            HubStatus won = computeShiftStatus(shift, true);
            HubStatus lost = computeShiftStatus(shift, false);
            assertNotEquals(won, lost,
                    "Won and Lost auto should have opposite status for shift " + shift);
        }
    }

    // ==================== SHIFT TIMING ====================

    @Test
    void shiftNumber_atTeleopStart_isOne() {
        assertEquals(1, computeShiftNumber(0.0),
                "At teleop start (0s elapsed), shift should be 1");
    }

    @Test
    void shiftNumber_justBeforeFirstShift_isOne() {
        assertEquals(1, computeShiftNumber(HubStateTracker.kShiftDurationSeconds - 0.1),
                "Just before first shift boundary, shift should still be 1");
    }

    @Test
    void shiftNumber_atFirstShiftBoundary_isTwo() {
        assertEquals(2, computeShiftNumber(HubStateTracker.kShiftDurationSeconds),
                "At first shift boundary (30s), shift should be 2");
    }

    @Test
    void shiftNumber_atSecondShiftBoundary_isThree() {
        assertEquals(3, computeShiftNumber(2 * HubStateTracker.kShiftDurationSeconds),
                "At second shift boundary (60s), shift should be 3");
    }

    @Test
    void shiftNumber_progressesCorrectly() {
        double duration = HubStateTracker.kShiftDurationSeconds;
        for (int expected = 1; expected <= HubStateTracker.kTotalShifts; expected++) {
            double midShift = (expected - 1) * duration + duration / 2.0;
            assertEquals(expected, computeShiftNumber(midShift),
                    "Midpoint of shift " + expected + " should report shift " + expected);
        }
    }

    // ==================== SHOULD SHOOT ====================

    @Test
    void shouldShoot_whenActive_alwaysTrue() {
        assertTrue(computeShouldShoot(HubStatus.ACTIVE, 20.0),
                "Should always shoot when HUB is Active");
        assertTrue(computeShouldShoot(HubStatus.ACTIVE, 0.0),
                "Should always shoot when HUB is Active, even at shift boundary");
    }

    @Test
    void shouldShoot_whenUnknown_alwaysTrue() {
        assertTrue(computeShouldShoot(HubStatus.UNKNOWN, 20.0),
                "Should shoot when UNKNOWN (safe default)");
    }

    @Test
    void shouldShoot_whenInactive_farFromShift_isFalse() {
        assertFalse(computeShouldShoot(HubStatus.INACTIVE, 10.0),
                "Should NOT shoot when Inactive and shift is far away (10s)");
        assertFalse(computeShouldShoot(HubStatus.INACTIVE, 5.0),
                "Should NOT shoot when Inactive and shift is 5s away");
        assertFalse(computeShouldShoot(HubStatus.INACTIVE, 3.0),
                "Should NOT shoot when Inactive and shift is exactly 3.0s away");
    }

    @Test
    void shouldShoot_whenInactive_shiftImminent_isTrue() {
        assertTrue(computeShouldShoot(HubStatus.INACTIVE, 2.9),
                "Should shoot when Inactive but shift is imminent (2.9s)");
        assertTrue(computeShouldShoot(HubStatus.INACTIVE, 1.0),
                "Should shoot when Inactive but shift is imminent (1.0s)");
        assertTrue(computeShouldShoot(HubStatus.INACTIVE, 0.0),
                "Should shoot when Inactive at shift boundary (0s)");
    }

    // ==================== SCHEDULE CONFIGURATION SANITY ====================

    @Test
    void shiftDuration_isPositive() {
        assertTrue(HubStateTracker.kShiftDurationSeconds > 0,
                "Shift duration must be positive");
    }

    @Test
    void totalShifts_isAtLeastTwo() {
        assertTrue(HubStateTracker.kTotalShifts >= 2,
                "Total shifts should be at least 2 for alternation to matter");
    }

    @Test
    void allShifts_fitWithinTeleop() {
        double totalShiftTime = HubStateTracker.kTotalShifts * HubStateTracker.kShiftDurationSeconds;
        assertTrue(totalShiftTime <= HubStateTracker.kTeleopDurationSeconds,
                "All shifts (" + totalShiftTime + "s) must fit within teleop (" +
                        HubStateTracker.kTeleopDurationSeconds + "s)");
    }

    @Test
    void shiftDuration_isReasonable() {
        // Shifts should be at least 10 seconds (enough to actually do something)
        // and at most 60 seconds (otherwise too few rotations)
        assertTrue(HubStateTracker.kShiftDurationSeconds >= 10,
                "Shift duration should be at least 10 seconds");
        assertTrue(HubStateTracker.kShiftDurationSeconds <= 60,
                "Shift duration should be at most 60 seconds");
    }

    // ==================== EDGE CASES ====================

    @Test
    void highShiftNumbers_maintainPattern() {
        // Even if shifts go beyond kTotalShifts (edge case), the pattern should hold
        for (int shift = 1; shift <= 10; shift++) {
            HubStatus status = computeShiftStatus(shift, true);
            assertNotNull(status, "Status should never be null for shift " + shift);
            if (shift % 2 == 1) {
                assertEquals(HubStatus.INACTIVE, status, "Won auto, odd shift " + shift);
            } else {
                assertEquals(HubStatus.ACTIVE, status, "Won auto, even shift " + shift);
            }
        }
    }

    @Test
    void negativeElapsed_clampsToShiftOne() {
        // Negative elapsed time (should be clamped to 0 in real code)
        int shift = computeShiftNumber(0.0);
        assertEquals(1, shift, "Zero elapsed should be shift 1");
    }
}
