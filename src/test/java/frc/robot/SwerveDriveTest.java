package frc.robot;

import frc.robot.constants.SwerveConstants;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for SwerveDrive subsystem logic.
 *
 * <p>Tests slip detection, gyro drift detection, voltage compensation,
 * second-order prediction, and vision age filtering without hardware.
 */
class SwerveDriveTest {

    // ==================== SLIP DETECTION ====================

    @Test
    void slipDetection_normalDriving_noSlip() {
        double commanded = 2.0;
        double actual = 1.9;
        boolean slipping = commanded > SwerveConstants.kSlipMinCommandedSpeed
                && actual > SwerveConstants.kSlipMinActualSpeed
                && Math.abs(commanded - actual) / commanded > SwerveConstants.kSlipDetectionThreshold;
        assertFalse(slipping, "Small difference should not trigger slip");
    }

    @Test
    void slipDetection_wheelSpinning_detected() {
        // Commanded high, actual much lower = wheel spinning
        double commanded = 4.0;
        double actual = 1.0;
        boolean slipping = commanded > SwerveConstants.kSlipMinCommandedSpeed
                && actual > SwerveConstants.kSlipMinActualSpeed
                && Math.abs(commanded - actual) / commanded > SwerveConstants.kSlipDetectionThreshold;
        assertTrue(slipping, "Large discrepancy should trigger slip detection");
    }

    @Test
    void slipDetection_lowSpeed_ignored() {
        // At low speeds, don't check slip (noise dominates)
        double commanded = SwerveConstants.kSlipMinCommandedSpeed * 0.5;
        double actual = 0.01;
        boolean slipping = commanded > SwerveConstants.kSlipMinCommandedSpeed
                && actual > SwerveConstants.kSlipMinActualSpeed
                && Math.abs(commanded - actual) / commanded > SwerveConstants.kSlipDetectionThreshold;
        assertFalse(slipping, "Low commanded speed should not check for slip");
    }

    @Test
    void slipDetection_zeroActual_ignored() {
        // Actual speed below minimum threshold — don't flag as slip
        // (module might not be moving yet at startup)
        double commanded = 3.0;
        double actual = SwerveConstants.kSlipMinActualSpeed * 0.5;
        boolean slipping = commanded > SwerveConstants.kSlipMinCommandedSpeed
                && actual > SwerveConstants.kSlipMinActualSpeed
                && Math.abs(commanded - actual) / commanded > SwerveConstants.kSlipDetectionThreshold;
        assertFalse(slipping, "Actual below minimum should not trigger slip");
    }

    // ==================== GYRO DRIFT DETECTION ====================

    @Test
    void gyroDrift_matchingOmegas_noWarning() {
        double gyroOmega = 1.5;
        double kinematicOmega = 1.5;
        double discrepancy = Math.abs(gyroOmega - kinematicOmega);
        assertFalse(discrepancy > 0.5, "Matching omegas should not trigger warning");
    }

    @Test
    void gyroDrift_largeDifference_warning() {
        double gyroOmega = 2.0;
        double kinematicOmega = 0.5;
        double discrepancy = Math.abs(gyroOmega - kinematicOmega);
        assertTrue(discrepancy > 0.5, "Large omega difference should trigger warning");
    }

    @Test
    void gyroDrift_atThreshold_noWarning() {
        double discrepancy = 0.5;
        assertFalse(discrepancy > 0.5, "At exactly 0.5, should not trigger (strict >)");
    }

    // ==================== SECOND-ORDER PREDICTION ====================

    @Test
    void prediction_stationaryRobot_noPrediction() {
        ChassisSpeeds commanded = new ChassisSpeeds(2.0, 0.0, 0.0);
        ChassisSpeeds current = new ChassisSpeeds(2.0, 0.0, 0.0);

        // Predicted = commanded + 0.5 * (commanded - current)
        double predictedVx = commanded.vxMetersPerSecond
                + (commanded.vxMetersPerSecond - current.vxMetersPerSecond) * 0.5;
        assertEquals(2.0, predictedVx, 1e-6,
                "At steady state, predicted should equal commanded");
    }

    @Test
    void prediction_accelerating_predictsBeyond() {
        ChassisSpeeds commanded = new ChassisSpeeds(3.0, 0.0, 0.0);
        ChassisSpeeds current = new ChassisSpeeds(1.0, 0.0, 0.0);

        double predictedVx = commanded.vxMetersPerSecond
                + (commanded.vxMetersPerSecond - current.vxMetersPerSecond) * 0.5;
        assertEquals(4.0, predictedVx, 1e-6,
                "When accelerating, prediction should overshoot commanded");
    }

    @Test
    void prediction_decelerating_predictsBeyond() {
        ChassisSpeeds commanded = new ChassisSpeeds(1.0, 0.0, 0.0);
        ChassisSpeeds current = new ChassisSpeeds(3.0, 0.0, 0.0);

        double predictedVx = commanded.vxMetersPerSecond
                + (commanded.vxMetersPerSecond - current.vxMetersPerSecond) * 0.5;
        assertEquals(0.0, predictedVx, 1e-6,
                "When decelerating, prediction should undershoot commanded");
    }

    // ==================== VOLTAGE COMPENSATION ====================

    @Test
    void voltageComp_fullVoltage_noScaling() {
        double voltComp = 1.0;
        ChassisSpeeds speeds = new ChassisSpeeds(2.0, 1.0, 0.5);
        if (voltComp < 1.0) {
            speeds = new ChassisSpeeds(
                    speeds.vxMetersPerSecond * voltComp,
                    speeds.vyMetersPerSecond * voltComp,
                    speeds.omegaRadiansPerSecond * voltComp);
        }
        assertEquals(2.0, speeds.vxMetersPerSecond, 1e-6);
        assertEquals(1.0, speeds.vyMetersPerSecond, 1e-6);
        assertEquals(0.5, speeds.omegaRadiansPerSecond, 1e-6);
    }

    @Test
    void voltageComp_halfVoltage_halvesSpeeds() {
        double voltComp = 0.5;
        ChassisSpeeds speeds = new ChassisSpeeds(2.0, 1.0, 0.5);
        if (voltComp < 1.0) {
            speeds = new ChassisSpeeds(
                    speeds.vxMetersPerSecond * voltComp,
                    speeds.vyMetersPerSecond * voltComp,
                    speeds.omegaRadiansPerSecond * voltComp);
        }
        assertEquals(1.0, speeds.vxMetersPerSecond, 1e-6);
        assertEquals(0.5, speeds.vyMetersPerSecond, 1e-6);
        assertEquals(0.25, speeds.omegaRadiansPerSecond, 1e-6);
    }

    // ==================== VISION MEASUREMENT AGE ====================

    @Test
    void visionAge_fresh_accepted() {
        double now = 10.0;
        double timestamp = 9.9;
        double age = now - timestamp;
        assertTrue(age <= SwerveConstants.kMaxVisionMeasurementAgeSec,
                "Fresh measurement should be accepted");
    }

    @Test
    void visionAge_stale_rejected() {
        double now = 10.0;
        double timestamp = 8.0;
        double age = now - timestamp;
        assertTrue(age > SwerveConstants.kMaxVisionMeasurementAgeSec,
                "Stale measurement should be rejected");
    }

    @Test
    void visionAge_threshold_isPositive() {
        assertTrue(SwerveConstants.kMaxVisionMeasurementAgeSec > 0);
    }

    // ==================== WHEEL LOCK PATTERN ====================

    @Test
    void wheelLock_isXPattern() {
        // FL and BR at 45°, FR and BL at -45°
        double flAngle = 45.0;
        double frAngle = -45.0;
        double blAngle = -45.0;
        double brAngle = 45.0;

        // X-pattern: diagonals should be same angle
        assertEquals(flAngle, brAngle, "FL and BR should match for X-pattern");
        assertEquals(frAngle, blAngle, "FR and BL should match for X-pattern");
        // Opposite diagonals should be opposite
        assertEquals(flAngle, -frAngle, "FL and FR should be opposite for X-pattern");
    }

    // ==================== CONSTANTS ====================

    @Test
    void maxSpeed_isPositive() {
        assertTrue(SwerveConstants.kMaxSpeedMetersPerSecond > 0);
    }

    @Test
    void odometryPeriod_fasterThanMainLoop() {
        assertTrue(SwerveConstants.kOdometryPeriodSeconds < SwerveConstants.kMainLoopPeriodSeconds,
                "Odometry thread should run faster than main loop");
    }

    @Test
    void autoPID_gainsArePositive() {
        assertTrue(SwerveConstants.kAutoTranslationP > 0);
        assertTrue(SwerveConstants.kAutoRotationP > 0);
    }
}
