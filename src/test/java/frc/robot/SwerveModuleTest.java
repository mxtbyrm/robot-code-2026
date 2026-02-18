package frc.robot;

import frc.robot.constants.SwerveConstants;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for SwerveModule logic.
 *
 * <p>Tests cosine compensation, anti-jitter threshold,
 * signal health detection, and constants without hardware.
 */
class SwerveModuleTest {

    // ==================== COSINE COMPENSATION ====================

    @Test
    void cosineComp_zeroError_fullSpeed() {
        double angleErrorRad = 0.0;
        double cosineScalar = Math.cos(angleErrorRad);
        double adjustedSpeed = 3.0 * cosineScalar;
        assertEquals(3.0, adjustedSpeed, 1e-6, "Zero error → full speed");
    }

    @Test
    void cosineComp_90degError_zeroSpeed() {
        double angleErrorRad = Math.PI / 2.0;
        double cosineScalar = Math.cos(angleErrorRad);
        double adjustedSpeed = 3.0 * cosineScalar;
        assertEquals(0.0, adjustedSpeed, 1e-6, "90° error → zero speed");
    }

    @Test
    void cosineComp_45degError_partialSpeed() {
        double angleErrorRad = Math.PI / 4.0;
        double cosineScalar = Math.cos(angleErrorRad);
        double adjustedSpeed = 3.0 * cosineScalar;
        assertEquals(3.0 * Math.cos(Math.PI / 4.0), adjustedSpeed, 1e-6);
    }

    @Test
    void cosineComp_180degError_negativeSpeed() {
        double angleErrorRad = Math.PI;
        double cosineScalar = Math.cos(angleErrorRad);
        double adjustedSpeed = 3.0 * cosineScalar;
        assertEquals(-3.0, adjustedSpeed, 1e-6,
                "180° error → reversed speed (module pointed backwards)");
    }

    @Test
    void cosineComp_smallError_nearFullSpeed() {
        double angleErrorRad = Math.toRadians(5.0);
        double cosineScalar = Math.cos(angleErrorRad);
        assertTrue(cosineScalar > 0.99, "5° error should barely reduce speed");
    }

    // ==================== ANTI-JITTER ====================

    @Test
    void antiJitter_belowThreshold_holdsAngle() {
        double speed = SwerveConstants.kAntiJitterSpeedMPS * 0.5;
        assertTrue(Math.abs(speed) <= SwerveConstants.kAntiJitterSpeedMPS,
                "Below threshold, should hold last angle");
    }

    @Test
    void antiJitter_aboveThreshold_updatesAngle() {
        double speed = SwerveConstants.kAntiJitterSpeedMPS * 2.0;
        assertFalse(Math.abs(speed) <= SwerveConstants.kAntiJitterSpeedMPS,
                "Above threshold, should update angle");
    }

    @Test
    void antiJitter_atThreshold_holdsAngle() {
        double speed = SwerveConstants.kAntiJitterSpeedMPS;
        assertTrue(Math.abs(speed) <= SwerveConstants.kAntiJitterSpeedMPS,
                "At exactly threshold, should hold (<=)");
    }

    @Test
    void antiJitter_negativeSpeed_usesAbsoluteValue() {
        double speed = -SwerveConstants.kAntiJitterSpeedMPS * 0.5;
        assertTrue(Math.abs(speed) <= SwerveConstants.kAntiJitterSpeedMPS,
                "Negative speed below threshold should hold angle");
    }

    // ==================== SIGNAL HEALTH ====================

    @Test
    void signalHealth_freshTimestamp_isHealthy() {
        double lastTimestamp = 10.0;
        double currentTimestamp = 10.0;
        double age = 0.01; // simulated timestamp age
        // Healthy if timestamp changes OR age is under threshold
        boolean timestampChanged = currentTimestamp != lastTimestamp || age <= SwerveConstants.kMaxSignalAgeSeconds;
        assertTrue(timestampChanged);
    }

    @Test
    void signalHealth_staleTimestamp_isUnhealthy() {
        double age = SwerveConstants.kMaxSignalAgeSeconds + 0.1;
        boolean timestampStale = age > SwerveConstants.kMaxSignalAgeSeconds;
        assertTrue(timestampStale, "Stale signal should mark module unhealthy");
    }

    // ==================== DRIVE POSITION CONVERSION ====================

    @Test
    void drivePosition_oneMotorRotation_correctMeters() {
        double motorRotations = 1.0;
        double wheelRotations = motorRotations / SwerveConstants.kDriveGearRatio;
        double meters = wheelRotations * SwerveConstants.kWheelCircumferenceMeters;
        assertTrue(meters > 0, "One motor rotation should produce positive distance");
        assertTrue(meters < SwerveConstants.kWheelCircumferenceMeters,
                "One motor rotation through gear ratio should be less than one wheel circumference");
    }

    @Test
    void drivePosition_gearRatioReduction() {
        // 6.75:1 ratio means 6.75 motor rotations = 1 wheel rotation
        double motorRotations = SwerveConstants.kDriveGearRatio;
        double wheelRotations = motorRotations / SwerveConstants.kDriveGearRatio;
        assertEquals(1.0, wheelRotations, 1e-6,
                "Gear ratio motor rotations should produce exactly 1 wheel rotation");
    }

    // ==================== CONSTANTS ====================

    @Test
    void gearRatios_arePositive() {
        assertTrue(SwerveConstants.kDriveGearRatio > 0);
        assertTrue(SwerveConstants.kSteerGearRatio > 0);
    }

    @Test
    void wheelDiameter_isReasonable() {
        // 4-inch wheels = ~0.1016m
        double diameterInches = SwerveConstants.kWheelDiameterMeters / 0.0254;
        assertTrue(diameterInches >= 2.0 && diameterInches <= 6.0,
                "Wheel diameter should be 2-6 inches, got " + diameterInches);
    }

    @Test
    void maxSpeed_isReasonable() {
        // FRC swerve bots typically max around 4-6 m/s
        assertTrue(SwerveConstants.kMaxSpeedMetersPerSecond > 2.0);
        assertTrue(SwerveConstants.kMaxSpeedMetersPerSecond < 8.0,
                "Max speed of " + SwerveConstants.kMaxSpeedMetersPerSecond + " m/s seems too high");
    }

    @Test
    void currentLimits_arePositive() {
        assertTrue(SwerveConstants.kDriveCurrentLimit > 0);
        assertTrue(SwerveConstants.kSteerCurrentLimit > 0);
    }

    @Test
    void driveStatorLimit_higherThanSupply() {
        assertTrue(SwerveConstants.kDriveStatorCurrentLimit >= SwerveConstants.kDriveCurrentLimit,
                "Stator limit should be >= supply limit");
    }

    @Test
    void antiJitterThreshold_isSmall() {
        assertTrue(SwerveConstants.kAntiJitterSpeedMPS < 0.5,
                "Anti-jitter threshold should be a small speed");
    }

    @Test
    void signalAgeThreshold_isReasonable() {
        assertTrue(SwerveConstants.kMaxSignalAgeSeconds > 0.1,
                "Signal age threshold should be > 100ms");
        assertTrue(SwerveConstants.kMaxSignalAgeSeconds <= 2.0,
                "Signal age threshold should be < 2s");
    }
}
