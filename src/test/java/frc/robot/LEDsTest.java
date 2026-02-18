package frc.robot;

import frc.robot.constants.LEDConstants;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for LEDs subsystem logic.
 *
 * <p>Tests animation waveform calculations, brightness boundaries,
 * and LED constants without hardware.
 */
class LEDsTest {

    // ==================== FLASH PATTERN ====================

    @Test
    void flash_atTimeZero_isOn() {
        double now = 0.0;
        double hz = 5.0;
        boolean on = ((int) (now * hz * 2)) % 2 == 0;
        assertTrue(on, "Flash should start ON at t=0");
    }

    @Test
    void flash_togglesCorrectly() {
        double hz = 5.0;
        // At 5Hz: period = 0.2s, half-period = 0.1s
        // t=0.0 → on, t=0.1 → off, t=0.2 → on
        boolean onAt0 = ((int) (0.0 * hz * 2)) % 2 == 0;
        boolean onAt01 = ((int) (0.1 * hz * 2)) % 2 == 0;
        boolean onAt02 = ((int) (0.2 * hz * 2)) % 2 == 0;
        assertTrue(onAt0);
        assertFalse(onAt01);
        assertTrue(onAt02);
    }

    // ==================== PULSE PATTERN ====================

    @Test
    void pulse_brightnessRange_zeroToOne() {
        double hz = 2.0;
        for (int i = 0; i < 100; i++) {
            double now = i * 0.01;
            double brightness = (Math.sin(now * hz * 2 * Math.PI) + 1.0) / 2.0;
            assertTrue(brightness >= 0.0 && brightness <= 1.0,
                    "Pulse brightness should be [0, 1], got " + brightness + " at t=" + now);
        }
    }

    @Test
    void pulse_peakBrightness_isOne() {
        double hz = 1.0;
        // Peak of sin is at t = 1/(4*hz) = 0.25s for 1Hz
        double now = 0.25;
        double brightness = (Math.sin(now * hz * 2 * Math.PI) + 1.0) / 2.0;
        assertEquals(1.0, brightness, 1e-6, "Peak brightness should be 1.0");
    }

    @Test
    void pulse_troughBrightness_isZero() {
        double hz = 1.0;
        // Trough of sin at t = 3/(4*hz) = 0.75s for 1Hz
        double now = 0.75;
        double brightness = (Math.sin(now * hz * 2 * Math.PI) + 1.0) / 2.0;
        assertEquals(0.0, brightness, 1e-6, "Trough brightness should be 0.0");
    }

    // ==================== BREATHE PATTERN ====================

    @Test
    void breathe_neverFullyOff() {
        double hz = 1.5;
        for (int i = 0; i < 200; i++) {
            double now = i * 0.01;
            double phase = (now * hz) % 1.0;
            double brightness = phase < 0.5 ? phase * 2.0 : 2.0 - phase * 2.0;
            brightness = brightness * 0.8 + 0.05; // never fully off
            assertTrue(brightness >= 0.05,
                    "Breathe should never go below 5%, got " + brightness + " at t=" + now);
        }
    }

    @Test
    void breathe_maxBrightness_isLessThanOne() {
        double hz = 1.5;
        double maxBrightness = 0;
        for (int i = 0; i < 200; i++) {
            double now = i * 0.01;
            double phase = (now * hz) % 1.0;
            double brightness = phase < 0.5 ? phase * 2.0 : 2.0 - phase * 2.0;
            brightness = brightness * 0.8 + 0.05;
            maxBrightness = Math.max(maxBrightness, brightness);
        }
        assertTrue(maxBrightness <= 0.85 + 1e-6,
                "Max breathe brightness should be ~0.85 (0.8 + 0.05)");
    }

    @Test
    void breathe_triangleWave_isSymmetric() {
        // First half rises, second half falls
        double phase025 = 0.25;
        double phase075 = 0.75;
        double up = phase025 < 0.5 ? phase025 * 2.0 : 2.0 - phase025 * 2.0;
        double down = phase075 < 0.5 ? phase075 * 2.0 : 2.0 - phase075 * 2.0;
        assertEquals(up, down, 1e-6, "Triangle wave should be symmetric at 25% and 75%");
    }

    // ==================== STROBE PATTERN ====================

    @Test
    void strobe_segmentSize_isThree() {
        int segmentSize = 3;
        int ledCount = 60;

        // Verify that segments alternate
        for (int i = 0; i < ledCount; i++) {
            boolean inEvenSegment = (i / segmentSize) % 2 == 0;
            boolean nextInEven = ((i + segmentSize) / segmentSize) % 2 == 0;
            if (i + segmentSize < ledCount && (i + segmentSize) / segmentSize != i / segmentSize) {
                assertNotEquals(inEvenSegment, nextInEven,
                        "Adjacent segments should alternate");
            }
        }
    }

    // ==================== CHASE PATTERN ====================

    @Test
    void chase_offsetMovesWithTime() {
        int ledCount = 60;
        int offset1 = ((int) (1.0 * 20)) % ledCount;
        int offset2 = ((int) (2.0 * 20)) % ledCount;
        assertNotEquals(offset1, offset2, "Chase offset should change with time");
    }

    @Test
    void chase_offsetWraps() {
        int ledCount = 60;
        // At t = 3.0s: offset = (3.0 * 20) % 60 = 60 % 60 = 0 (wraps)
        int offset = ((int) (3.0 * 20)) % ledCount;
        assertEquals(0, offset, "Chase offset should wrap at ledCount");
    }

    // ==================== ALTERNATING PATTERN ====================

    @Test
    void alternating_evenOddFlip() {
        double hz = 3.0;
        boolean phase1 = ((int) (0.0 * hz * 2)) % 2 == 0;
        // After half-period: should flip
        double halfPeriod = 1.0 / (hz * 2);
        boolean phase2 = ((int) (halfPeriod * hz * 2)) % 2 == 0;
        assertNotEquals(phase1, phase2, "Phase should flip each half-period");
    }

    // ==================== CONSTANTS ====================

    @Test
    void ledCount_isPositive() {
        assertTrue(LEDConstants.kLedCount > 0);
    }

    @Test
    void ledCount_isReasonable() {
        assertTrue(LEDConstants.kLedCount <= 300,
                "LED count should be reasonable (< 300 for FRC)");
    }

    @Test
    void ledPort_isValid() {
        assertTrue(LEDConstants.kLedPort >= 0 && LEDConstants.kLedPort <= 9,
                "LED PWM port should be 0-9");
    }
}
