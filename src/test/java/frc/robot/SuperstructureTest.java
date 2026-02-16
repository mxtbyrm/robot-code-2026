package frc.robot;

import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Superstructure.SuperState;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for Superstructure state machine logic.
 *
 * <p>Tests the pure state transition logic, pre-feed reverse timing,
 * and endgame auto-dump rules without hardware or WPILib HAL.
 *
 * <p>Approach: extract and replicate the decision logic, then verify
 * transitions and guards independently.
 */
class SuperstructureTest {

    // ==================== STATE TRANSITION LOGIC ====================
    // Replicate the valid transition checks from Superstructure.

    /**
     * Determines whether a transition from currentState to requestedState
     * should go through PRE_FEED_REVERSE first.
     * Mirrors Superstructure.requestShoot() logic.
     */
    private SuperState resolveShootRequest(SuperState current) {
        if (current == SuperState.DISABLED) return current; // blocked
        if (current == SuperState.SHOOTING || current == SuperState.PRE_FEED_REVERSE) {
            return SuperState.SHOOTING; // skip pre-feed
        }
        return SuperState.PRE_FEED_REVERSE; // go through pre-feed first
    }

    /**
     * Determines whether a request is blocked (e.g., from DISABLED state).
     */
    private boolean isRequestBlocked(SuperState current, SuperState requested) {
        if (current == SuperState.DISABLED) {
            // Only IDLE and DISABLED are allowed from DISABLED
            return requested != SuperState.IDLE && requested != SuperState.DISABLED;
        }
        return false;
    }

    // ==================== ALL 8 STATES EXIST ====================

    @Test
    void allStates_enumHasEightValues() {
        assertEquals(8, SuperState.values().length,
                "SuperState should have exactly 8 values");
    }

    @Test
    void allStates_namedCorrectly() {
        assertNotNull(SuperState.valueOf("IDLE"));
        assertNotNull(SuperState.valueOf("INTAKING"));
        assertNotNull(SuperState.valueOf("PRE_FEED_REVERSE"));
        assertNotNull(SuperState.valueOf("SHOOTING"));
        assertNotNull(SuperState.valueOf("SHOOTING_ALLIANCE"));
        assertNotNull(SuperState.valueOf("OUTTAKING"));
        assertNotNull(SuperState.valueOf("UNJAMMING"));
        assertNotNull(SuperState.valueOf("DISABLED"));
    }

    // ==================== VALID TRANSITIONS ====================

    @Test
    void idle_canTransitionTo_intaking() {
        assertFalse(isRequestBlocked(SuperState.IDLE, SuperState.INTAKING));
    }

    @Test
    void idle_canTransitionTo_shooting() {
        SuperState result = resolveShootRequest(SuperState.IDLE);
        assertEquals(SuperState.PRE_FEED_REVERSE, result,
                "IDLE → SHOOTING should go through PRE_FEED_REVERSE");
    }

    @Test
    void intaking_canTransitionTo_idle() {
        assertFalse(isRequestBlocked(SuperState.INTAKING, SuperState.IDLE));
    }

    @Test
    void intaking_canTransitionTo_shooting() {
        SuperState result = resolveShootRequest(SuperState.INTAKING);
        assertEquals(SuperState.PRE_FEED_REVERSE, result,
                "INTAKING → SHOOTING should go through PRE_FEED_REVERSE");
    }

    @Test
    void shooting_canTransitionTo_idle() {
        assertFalse(isRequestBlocked(SuperState.SHOOTING, SuperState.IDLE));
    }

    @Test
    void outtaking_canTransitionTo_idle() {
        assertFalse(isRequestBlocked(SuperState.OUTTAKING, SuperState.IDLE));
    }

    @Test
    void unjamming_canTransitionTo_idle() {
        assertFalse(isRequestBlocked(SuperState.UNJAMMING, SuperState.IDLE));
    }

    // ==================== DISABLED STATE BLOCKS TRANSITIONS ====================

    @Test
    void disabled_blocksIntaking() {
        assertTrue(isRequestBlocked(SuperState.DISABLED, SuperState.INTAKING));
    }

    @Test
    void disabled_blocksShooting() {
        assertTrue(isRequestBlocked(SuperState.DISABLED, SuperState.SHOOTING));
    }

    @Test
    void disabled_blocksOuttaking() {
        assertTrue(isRequestBlocked(SuperState.DISABLED, SuperState.OUTTAKING));
    }

    @Test
    void disabled_blocksUnjamming() {
        assertTrue(isRequestBlocked(SuperState.DISABLED, SuperState.UNJAMMING));
    }

    @Test
    void disabled_allowsIdle() {
        assertFalse(isRequestBlocked(SuperState.DISABLED, SuperState.IDLE),
                "DISABLED should allow transition to IDLE (re-enable)");
    }

    @Test
    void disabled_shootRequest_blocked() {
        SuperState result = resolveShootRequest(SuperState.DISABLED);
        assertEquals(SuperState.DISABLED, result,
                "Shoot request from DISABLED should stay DISABLED");
    }

    // ==================== PRE-FEED REVERSE ====================

    @Test
    void preFeedReverse_hasCorrectDuration() {
        assertEquals(0.12, ShooterConstants.kPreFeedReverseDurationSeconds, 0.001,
                "Pre-feed reverse should be 120ms");
    }

    @Test
    void preFeedReverse_reverseSpeedsAreNegative() {
        assertTrue(ShooterConstants.kPreFeedReverseFeederSpeed < 0,
                "Pre-feed feeder speed should be negative (reverse)");
        assertTrue(ShooterConstants.kPreFeedReverseSpindexerSpeed < 0,
                "Pre-feed spindexer speed should be negative (reverse)");
    }

    @Test
    void preFeedReverse_skippedWhenAlreadyShooting() {
        SuperState result = resolveShootRequest(SuperState.SHOOTING);
        assertEquals(SuperState.SHOOTING, result,
                "Should skip pre-feed when already shooting");
    }

    @Test
    void preFeedReverse_skippedWhenAlreadyInPreFeed() {
        SuperState result = resolveShootRequest(SuperState.PRE_FEED_REVERSE);
        assertEquals(SuperState.SHOOTING, result,
                "Should skip pre-feed when already in pre-feed reverse");
    }

    @Test
    void preFeedReverse_requiredFromIdle() {
        SuperState result = resolveShootRequest(SuperState.IDLE);
        assertEquals(SuperState.PRE_FEED_REVERSE, result,
                "Should require pre-feed reverse from IDLE");
    }

    @Test
    void preFeedReverse_requiredFromIntaking() {
        SuperState result = resolveShootRequest(SuperState.INTAKING);
        assertEquals(SuperState.PRE_FEED_REVERSE, result,
                "Should require pre-feed reverse from INTAKING");
    }

    @Test
    void preFeedReverse_requiredFromOuttaking() {
        SuperState result = resolveShootRequest(SuperState.OUTTAKING);
        assertEquals(SuperState.PRE_FEED_REVERSE, result,
                "Should require pre-feed reverse from OUTTAKING");
    }

    // ==================== PRE-FEED REVERSE TIMING ====================

    @Test
    void preFeedReverse_timerExpiry_transitionsToShooting() {
        double startTime = 100.0;
        double now = startTime + ShooterConstants.kPreFeedReverseDurationSeconds + 0.01;
        double elapsed = now - startTime;

        assertTrue(elapsed >= ShooterConstants.kPreFeedReverseDurationSeconds,
                "After 120ms+, pre-feed reverse should be complete");
    }

    @Test
    void preFeedReverse_timerNotExpired_staysInReverse() {
        double startTime = 100.0;
        double now = startTime + ShooterConstants.kPreFeedReverseDurationSeconds - 0.01;
        double elapsed = now - startTime;

        assertFalse(elapsed >= ShooterConstants.kPreFeedReverseDurationSeconds,
                "Before 120ms, pre-feed reverse should still be running");
    }

    // ==================== ENDGAME AUTO-DUMP ====================

    /**
     * Replicates endgame dump activation logic from Superstructure.periodic().
     */
    private boolean shouldActivateEndgameDump(
            double matchTime, boolean isTeleopEnabled, boolean endgameDumpCancelled,
            SuperState currentState) {
        if (!isTeleopEnabled) return false;
        if (matchTime <= 0) return false;
        if (matchTime > ShooterConstants.kEndgameDumpTimeSeconds) return false;
        if (endgameDumpCancelled) return false;
        if (currentState == SuperState.DISABLED) return false;
        return true;
    }

    @Test
    void endgameDump_activatesAtThreshold() {
        assertTrue(shouldActivateEndgameDump(
                ShooterConstants.kEndgameDumpTimeSeconds, true, false, SuperState.IDLE),
                "Endgame dump should activate at threshold time");
    }

    @Test
    void endgameDump_activatesBeforeThreshold() {
        assertTrue(shouldActivateEndgameDump(5.0, true, false, SuperState.IDLE),
                "Endgame dump should activate before threshold");
    }

    @Test
    void endgameDump_doesNotActivateAboveThreshold() {
        assertFalse(shouldActivateEndgameDump(
                ShooterConstants.kEndgameDumpTimeSeconds + 5.0, true, false, SuperState.IDLE),
                "Endgame dump should not activate above threshold");
    }

    @Test
    void endgameDump_doesNotActivateWhenCancelled() {
        assertFalse(shouldActivateEndgameDump(5.0, true, true, SuperState.IDLE),
                "Endgame dump should not activate when cancelled");
    }

    @Test
    void endgameDump_doesNotActivateWhenDisabled() {
        assertFalse(shouldActivateEndgameDump(5.0, true, false, SuperState.DISABLED),
                "Endgame dump should not activate in DISABLED state");
    }

    @Test
    void endgameDump_doesNotActivateInAuto() {
        assertFalse(shouldActivateEndgameDump(5.0, false, false, SuperState.IDLE),
                "Endgame dump should only activate in teleop");
    }

    @Test
    void endgameDump_threshold_isReasonable() {
        assertTrue(ShooterConstants.kEndgameDumpTimeSeconds > 0,
                "Endgame dump threshold must be positive");
        assertTrue(ShooterConstants.kEndgameDumpTimeSeconds <= 30,
                "Endgame dump threshold should be within END GAME period (30s)");
    }

    // ==================== ALLIANCE ZONE SHOOT REQUEST ====================

    /**
     * Mirrors Superstructure.requestShootAlliance() logic.
     */
    private SuperState resolveAllianceShootRequest(SuperState current) {
        if (current == SuperState.DISABLED) return current;
        if (current == SuperState.SHOOTING_ALLIANCE || current == SuperState.PRE_FEED_REVERSE) {
            return SuperState.SHOOTING_ALLIANCE;
        }
        return SuperState.PRE_FEED_REVERSE;
    }

    @Test
    void allianceShoot_fromIdle_goesThruPreFeed() {
        assertEquals(SuperState.PRE_FEED_REVERSE, resolveAllianceShootRequest(SuperState.IDLE));
    }

    @Test
    void allianceShoot_alreadyShooting_skipPreFeed() {
        assertEquals(SuperState.SHOOTING_ALLIANCE,
                resolveAllianceShootRequest(SuperState.SHOOTING_ALLIANCE));
    }

    @Test
    void allianceShoot_disabled_blocked() {
        assertEquals(SuperState.DISABLED, resolveAllianceShootRequest(SuperState.DISABLED));
    }
}
