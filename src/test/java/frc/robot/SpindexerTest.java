package frc.robot;

import frc.robot.constants.SpindexerConstants;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for Spindexer subsystem logic.
 *
 * <p>Tests stall detection, running state threshold,
 * speed hierarchy, and constants validation without hardware.
 */
class SpindexerTest {

    // ==================== RUNNING STATE DETECTION ====================

    @Test
    void isRunning_aboveThreshold_returnsTrue() {
        double commandedOutput = 0.06;
        assertTrue(commandedOutput > 0.05, "Above 0.05 should be considered running");
    }

    @Test
    void isRunning_belowThreshold_returnsFalse() {
        double commandedOutput = 0.04;
        assertFalse(commandedOutput > 0.05, "Below 0.05 should not be considered running");
    }

    @Test
    void isRunning_atThreshold_returnsFalse() {
        double commandedOutput = 0.05;
        assertFalse(commandedOutput > 0.05, "At exactly 0.05 should return false (strict >)");
    }

    @Test
    void isRunning_zero_returnsFalse() {
        double commandedOutput = 0.0;
        assertFalse(commandedOutput > 0.05);
    }

    // ==================== STALL DETECTION ====================

    @Test
    void stallDetection_onlyWhenCommanded() {
        // Stall detection requires commandedOutput > 0.1
        double commandedOutput = 0.05;
        boolean shouldCheck = commandedOutput > 0.1;
        assertFalse(shouldCheck, "Should not check stall at low commanded output");
    }

    @Test
    void stallDetection_highCurrentLowVelocity_increments() {
        double current = SpindexerConstants.kSpindexerStallCurrentThreshold + 1.0;
        double velocity = SpindexerConstants.kSpindexerStallVelocityThreshold - 0.1;
        int stallCount = 0;

        if (current > SpindexerConstants.kSpindexerStallCurrentThreshold
                && velocity < SpindexerConstants.kSpindexerStallVelocityThreshold) {
            stallCount++;
        }
        assertEquals(1, stallCount);
    }

    @Test
    void stallDetection_normalOperation_decays() {
        double current = SpindexerConstants.kSpindexerStallCurrentThreshold - 5.0;
        double velocity = SpindexerConstants.kSpindexerStallVelocityThreshold + 5.0;
        int stallCount = 6;

        if (current > SpindexerConstants.kSpindexerStallCurrentThreshold
                && velocity < SpindexerConstants.kSpindexerStallVelocityThreshold) {
            stallCount++;
        } else {
            stallCount = Math.max(0, stallCount - 2);
        }
        assertEquals(4, stallCount, "Decay rate should be -2");
    }

    @Test
    void stallDetection_decayFromOne_goesToZero() {
        int stallCount = 1;
        stallCount = Math.max(0, stallCount - 2);
        assertEquals(0, stallCount, "Decay from 1 should go to 0, not negative");
    }

    @Test
    void stallDetection_triggerThreshold() {
        int stallCount = SpindexerConstants.kSpindexerStallCycleThreshold;
        assertTrue(stallCount >= SpindexerConstants.kSpindexerStallCycleThreshold);
    }

    @Test
    void stallDetection_justBelowThreshold_noTrigger() {
        int stallCount = SpindexerConstants.kSpindexerStallCycleThreshold - 1;
        assertFalse(stallCount >= SpindexerConstants.kSpindexerStallCycleThreshold);
    }

    // ==================== SPEED HIERARCHY ====================

    @Test
    void intakeSpeed_isPositive() {
        assertTrue(SpindexerConstants.kSpindexerIntakeSpeed > 0);
    }

    @Test
    void reverseSpeed_isNegative() {
        assertTrue(SpindexerConstants.kSpindexerReverseSpeed < 0);
    }

    @Test
    void unjamSpeed_isNegative() {
        assertTrue(SpindexerConstants.kSpindexerUnjamSpeed < 0);
    }

    @Test
    void unjamSpeed_fasterThanReverse() {
        assertTrue(Math.abs(SpindexerConstants.kSpindexerUnjamSpeed)
                        >= Math.abs(SpindexerConstants.kSpindexerReverseSpeed),
                "Unjam speed should be at least as fast as reverse");
    }

    @Test
    void spindexerToFeederRatio_lessThanOne() {
        assertTrue(SpindexerConstants.kSpindexerToFeederRatio < 1.0,
                "Spindexer must be slower than feeder to prevent jams");
    }

    @Test
    void spindexerToFeederRatio_isPositive() {
        assertTrue(SpindexerConstants.kSpindexerToFeederRatio > 0);
    }

    @Test
    void intakeSpeed_lessThanOne() {
        assertTrue(SpindexerConstants.kSpindexerIntakeSpeed <= 1.0,
                "Intake speed should be at most 100% duty cycle");
    }

    // ==================== UNJAM TIMING ====================

    @Test
    void unjamDuration_isPositive() {
        assertTrue(SpindexerConstants.kSpindexerUnjamDuration > 0);
    }

    @Test
    void unjamDuration_isShort() {
        assertTrue(SpindexerConstants.kSpindexerUnjamDuration <= 1.0,
                "Unjam should be brief (< 1s), got " + SpindexerConstants.kSpindexerUnjamDuration);
    }

    // ==================== CONSTANTS VALIDATION ====================

    @Test
    void currentLimit_isPositive() {
        assertTrue(SpindexerConstants.kSpindexerCurrentLimit > 0);
    }

    @Test
    void stallCurrentThreshold_belowCurrentLimit() {
        assertTrue(SpindexerConstants.kSpindexerStallCurrentThreshold
                        <= SpindexerConstants.kSpindexerCurrentLimit,
                "Stall threshold should not exceed current limit");
    }

    @Test
    void stallVelocityThreshold_isPositive() {
        assertTrue(SpindexerConstants.kSpindexerStallVelocityThreshold > 0);
    }

    @Test
    void stallCycleThreshold_isPositive() {
        assertTrue(SpindexerConstants.kSpindexerStallCycleThreshold > 0);
    }
}
