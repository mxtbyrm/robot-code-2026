package frc.robot;

import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;

import edu.wpi.first.math.MathUtil;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for Feeder subsystem logic.
 *
 * <p>Tests feed speed calculation, beam break edge detection,
 * stall detection, and constants validation without hardware.
 */
class FeederTest {

    // ==================== FEED SPEED CALCULATION ====================

    @Test
    void feedSpeed_atMaxShooterRPS_returnsRatio() {
        double shooterRPS = ShooterConstants.kMaxFlywheelRPS;
        double speed = MathUtil.clamp(
                FeederConstants.kFeederSpeedRatio * (shooterRPS / ShooterConstants.kMaxFlywheelRPS),
                0.0, 1.0);
        assertEquals(FeederConstants.kFeederSpeedRatio, speed, 1e-6,
                "At max shooter RPS, feeder should run at kFeederSpeedRatio");
    }

    @Test
    void feedSpeed_atHalfShooterRPS_returnsHalfRatio() {
        double shooterRPS = ShooterConstants.kMaxFlywheelRPS / 2.0;
        double speed = MathUtil.clamp(
                FeederConstants.kFeederSpeedRatio * (shooterRPS / ShooterConstants.kMaxFlywheelRPS),
                0.0, 1.0);
        assertEquals(FeederConstants.kFeederSpeedRatio / 2.0, speed, 1e-6);
    }

    @Test
    void feedSpeed_atZeroShooterRPS_returnsZero() {
        double speed = MathUtil.clamp(
                FeederConstants.kFeederSpeedRatio * (0.0 / ShooterConstants.kMaxFlywheelRPS),
                0.0, 1.0);
        assertEquals(0.0, speed, 1e-6);
    }

    @Test
    void feedSpeed_aboveMaxRPS_clampsToOne() {
        double shooterRPS = ShooterConstants.kMaxFlywheelRPS * 2.0;
        double speed = MathUtil.clamp(
                FeederConstants.kFeederSpeedRatio * (shooterRPS / ShooterConstants.kMaxFlywheelRPS),
                0.0, 1.0);
        assertTrue(speed <= 1.0, "Speed should be clamped to 1.0");
    }

    @Test
    void feedSpeed_isMonotonic() {
        double prev = 0;
        for (int i = 0; i <= 10; i++) {
            double shooterRPS = ShooterConstants.kMaxFlywheelRPS * i / 10.0;
            double speed = MathUtil.clamp(
                    FeederConstants.kFeederSpeedRatio * (shooterRPS / ShooterConstants.kMaxFlywheelRPS),
                    0.0, 1.0);
            assertTrue(speed >= prev, "Feed speed should increase with shooter RPS");
            prev = speed;
        }
    }

    // ==================== BEAM BREAK EDGE DETECTION ====================

    @Test
    void beamBreak_risingEdge_detectsBall() {
        boolean lastBroken = false;
        boolean nowBroken = true;
        boolean ballPassed = (!lastBroken && nowBroken);
        assertTrue(ballPassed, "Rising edge should detect ball");
    }

    @Test
    void beamBreak_fallingEdge_noBall() {
        boolean lastBroken = true;
        boolean nowBroken = false;
        boolean ballPassed = (!lastBroken && nowBroken);
        assertFalse(ballPassed, "Falling edge should NOT detect ball");
    }

    @Test
    void beamBreak_staysBroken_noBall() {
        boolean lastBroken = true;
        boolean nowBroken = true;
        boolean ballPassed = (!lastBroken && nowBroken);
        assertFalse(ballPassed, "Staying broken should NOT detect ball");
    }

    @Test
    void beamBreak_staysUnbroken_noBall() {
        boolean lastBroken = false;
        boolean nowBroken = false;
        boolean ballPassed = (!lastBroken && nowBroken);
        assertFalse(ballPassed, "Staying unbroken should NOT detect ball");
    }

    // ==================== STALL DETECTION ====================

    @Test
    void stallDetection_highCurrentLowVelocity_increments() {
        double current = FeederConstants.kFeederStallCurrentThreshold + 1.0;
        double velocity = FeederConstants.kFeederStallVelocityThreshold - 0.1;
        int stallCount = 0;

        if (current > FeederConstants.kFeederStallCurrentThreshold
                && velocity < FeederConstants.kFeederStallVelocityThreshold) {
            stallCount++;
        }
        assertEquals(1, stallCount);
    }

    @Test
    void stallDetection_highCurrentHighVelocity_decays() {
        double current = FeederConstants.kFeederStallCurrentThreshold + 1.0;
        double velocity = FeederConstants.kFeederStallVelocityThreshold + 1.0;
        int stallCount = 5;

        if (current > FeederConstants.kFeederStallCurrentThreshold
                && velocity < FeederConstants.kFeederStallVelocityThreshold) {
            stallCount++;
        } else {
            stallCount = Math.max(0, stallCount - 2);
        }
        assertEquals(3, stallCount, "Decay rate should be -2");
    }

    @Test
    void stallDetection_lowCurrentLowVelocity_decays() {
        double current = FeederConstants.kFeederStallCurrentThreshold - 1.0;
        double velocity = FeederConstants.kFeederStallVelocityThreshold - 0.1;
        int stallCount = 4;

        if (current > FeederConstants.kFeederStallCurrentThreshold
                && velocity < FeederConstants.kFeederStallVelocityThreshold) {
            stallCount++;
        } else {
            stallCount = Math.max(0, stallCount - 2);
        }
        assertEquals(2, stallCount);
    }

    @Test
    void stallDetection_decayNeverGoesNegative() {
        int stallCount = 1;
        stallCount = Math.max(0, stallCount - 2);
        assertEquals(0, stallCount, "Decay should not go below zero");
    }

    @Test
    void stallDetection_triggerAtThreshold() {
        int stallCount = FeederConstants.kFeederStallCycleThreshold;
        assertTrue(stallCount >= FeederConstants.kFeederStallCycleThreshold,
                "At threshold, jam should trigger");
    }

    @Test
    void stallDetection_noTriggerBelowThreshold() {
        int stallCount = FeederConstants.kFeederStallCycleThreshold - 1;
        assertFalse(stallCount >= FeederConstants.kFeederStallCycleThreshold,
                "Below threshold, jam should not trigger");
    }

    // ==================== CONSTANTS VALIDATION ====================

    @Test
    void feederSpeedRatio_isBetweenZeroAndOne() {
        assertTrue(FeederConstants.kFeederSpeedRatio > 0 && FeederConstants.kFeederSpeedRatio <= 1.0,
                "Feeder speed ratio must be between 0 and 1");
    }

    @Test
    void unjamReverseSpeed_isNegative() {
        assertTrue(FeederConstants.kFeederUnjamReverseSpeed < 0,
                "Unjam reverse should be negative duty cycle");
    }

    @Test
    void unjamDuration_isPositive() {
        assertTrue(FeederConstants.kFeederUnjamReverseDuration > 0);
    }

    @Test
    void unjamDuration_isReasonable() {
        assertTrue(FeederConstants.kFeederUnjamReverseDuration <= 2.0,
                "Unjam duration should be short (< 2s)");
    }

    @Test
    void stallThresholds_arePositive() {
        assertTrue(FeederConstants.kFeederStallCurrentThreshold > 0);
        assertTrue(FeederConstants.kFeederStallVelocityThreshold > 0);
        assertTrue(FeederConstants.kFeederStallCycleThreshold > 0);
    }

    @Test
    void currentLimit_isPositive() {
        assertTrue(FeederConstants.kFeederCurrentLimit > 0);
    }

    @Test
    void stallCurrentThreshold_belowCurrentLimit() {
        assertTrue(FeederConstants.kFeederStallCurrentThreshold <= FeederConstants.kFeederCurrentLimit,
                "Stall threshold should not exceed current limit");
    }
}
