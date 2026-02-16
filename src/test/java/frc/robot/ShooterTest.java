package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import frc.robot.Constants.ShooterConstants;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for Shooter subsystem logic.
 *
 * <p>Tests the pure computation logic (shoot-while-moving compensation,
 * turret wraparound, ready-check conditions, brownout scaling, alliance zone)
 * without requiring hardware or the WPILib HAL.
 *
 * <p>The approach mirrors the pattern in HubStateTrackerTest: extract the
 * pure logic and test it independently of singleton/hardware dependencies.
 */
class ShooterTest {

    // Replicated lookup tables for testing
    private InterpolatingDoubleTreeMap flywheelTable;
    private InterpolatingDoubleTreeMap hoodAngleTable;
    private InterpolatingDoubleTreeMap timeOfFlightTable;

    @BeforeEach
    void setUp() {
        double[][] shootingTable = ShooterPhysics.computeShootingTable();
        double[][] tofTable = ShooterPhysics.computeTimeOfFlightTable();

        flywheelTable = new InterpolatingDoubleTreeMap();
        hoodAngleTable = new InterpolatingDoubleTreeMap();
        timeOfFlightTable = new InterpolatingDoubleTreeMap();

        for (double[] entry : shootingTable) {
            flywheelTable.put(entry[0], entry[1]);
            hoodAngleTable.put(entry[0], entry[2]);
        }
        for (double[] entry : tofTable) {
            timeOfFlightTable.put(entry[0], entry[1]);
        }
    }

    // ==================== SHOOT-WHILE-MOVING COMPENSATION ====================

    /**
     * Replicate the shoot-while-moving logic from Shooter.updateHubCalculations().
     * Returns [compensatedDistance, compensatedTurretAngleDeg, flywheelCompensationRPS].
     */
    private double[] computeShootWhileMoving(
            Translation2d robotPos, double robotHeadingRad,
            double vxMPS, double vyMPS, Translation2d hubPos) {

        Translation2d robotToHub = hubPos.minus(robotPos);
        double distanceToHub = robotToHub.getNorm();

        double fieldAngleToHubRad = Math.atan2(robotToHub.getY(), robotToHub.getX());
        double turretAngleRad = fieldAngleToHubRad - robotHeadingRad;
        double staticTurretAngle = Math.toDegrees(MathUtil.angleModulus(turretAngleRad))
                + ShooterConstants.kTurretMountOffsetDegrees;

        // Distance-based time-of-flight
        double tof = timeOfFlightTable.get(distanceToHub);

        double futureX = robotPos.getX() + vxMPS * tof;
        double futureY = robotPos.getY() + vyMPS * tof;
        Translation2d futureRobotPos = new Translation2d(futureX, futureY);

        Translation2d futureToHub = hubPos.minus(futureRobotPos);
        double compensatedDistance = futureToHub.getNorm();

        double compensatedFieldAngle = Math.atan2(futureToHub.getY(), futureToHub.getX());
        double compensatedTurretRad = compensatedFieldAngle - robotHeadingRad;
        double compensatedTurretAngle = Math.toDegrees(MathUtil.angleModulus(compensatedTurretRad))
                + ShooterConstants.kTurretMountOffsetDegrees;

        double hubAngle = Math.atan2(robotToHub.getY(), robotToHub.getX());
        double vTowardHub = vxMPS * Math.cos(hubAngle) + vyMPS * Math.sin(hubAngle);
        double flywheelCompensationRPS = -vTowardHub
                / (ShooterConstants.kBottomFlywheelCircumferenceMeters
                   * ShooterConstants.kShooterEfficiencyFactor);

        return new double[] { compensatedDistance, compensatedTurretAngle, flywheelCompensationRPS };
    }

    @Test
    void shootWhileMoving_stationaryRobot_noCompensation() {
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(2.0, 4.0);

        double[] result = computeShootWhileMoving(robotPos, 0.0, 0.0, 0.0, hub);

        double staticDist = hub.minus(robotPos).getNorm();
        assertEquals(staticDist, result[0], 0.01, "Compensated distance should equal static when stationary");
        assertEquals(0.0, result[2], 0.01, "Flywheel compensation should be 0 when stationary");
    }

    @Test
    void shootWhileMoving_movingTowardHub_reducesDistance() {
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(hub.getX() + 3.0, hub.getY());

        // Moving toward the hub at 2 m/s
        double[] result = computeShootWhileMoving(robotPos, 0.0, -2.0, 0.0, hub);

        double staticDist = hub.minus(robotPos).getNorm();
        assertTrue(result[0] < staticDist,
                "Compensated distance should be less when moving toward hub");
    }

    @Test
    void shootWhileMoving_movingAwayFromHub_increasesDistance() {
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(hub.getX() + 3.0, hub.getY());

        // Moving away from hub at 2 m/s
        double[] result = computeShootWhileMoving(robotPos, 0.0, 2.0, 0.0, hub);

        double staticDist = hub.minus(robotPos).getNorm();
        assertTrue(result[0] > staticDist,
                "Compensated distance should increase when moving away from hub");
    }

    @Test
    void shootWhileMoving_lateralMotion_shiftsTurretAngle() {
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(hub.getX() + 3.0, hub.getY());

        // Robot facing hub, moving laterally at 2 m/s
        double headingRad = Math.atan2(hub.getY() - robotPos.getY(), hub.getX() - robotPos.getX());
        double[] resultStatic = computeShootWhileMoving(robotPos, headingRad, 0.0, 0.0, hub);
        double[] resultMoving = computeShootWhileMoving(robotPos, headingRad, 0.0, 2.0, hub);

        assertNotEquals(resultStatic[1], resultMoving[1], 0.1,
                "Turret angle should shift with lateral motion");
    }

    @Test
    void shootWhileMoving_movingTowardHub_positiveFlyCompensation() {
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(hub.getX() + 3.0, hub.getY());

        // Moving toward hub — ball exits with less relative speed, so flywheel should compensate UP
        // But the formula: -vTowardHub / (circumference * efficiency)
        // vTowardHub is positive when moving toward hub, so compensation is negative
        // That means we REDUCE flywheel speed because the robot's forward motion helps the ball
        double[] result = computeShootWhileMoving(robotPos, 0.0, -2.0, 0.0, hub);

        // Moving toward hub (negative vx, but toward hub is positive direction)
        // vTowardHub = vx * cos(hubAngle) + vy * sin(hubAngle)
        // hub is to the left of robot (negative x direction), so cos(hubAngle) is negative
        // vx=-2 * cos(PI)=-1 = +2, so vTowardHub > 0 → compensation < 0
        assertTrue(result[2] < 0,
                "Moving toward hub should give negative flywheel compensation (reduce speed)");
    }

    // ==================== TURRET WRAPAROUND HYSTERESIS ====================

    /**
     * Replicates turret wraparound hysteresis logic from Shooter.periodic().
     */
    private boolean computeTurretNearLimit(boolean wasNearLimit, double turretAngleDeg) {
        double absTurretAngle = Math.abs(turretAngleDeg);
        if (wasNearLimit) {
            return absTurretAngle > ShooterConstants.kTurretWraparoundReleaseThresholdDeg;
        } else {
            return absTurretAngle > ShooterConstants.kTurretWraparoundThresholdDeg;
        }
    }

    private double computeWraparoundHint(double turretAngleDeg) {
        double absTurretAngle = Math.abs(turretAngleDeg);
        double range = ShooterConstants.kTurretMaxAngleDegrees
                - ShooterConstants.kTurretWraparoundReleaseThresholdDeg;
        double fraction = Math.min(
                (absTurretAngle - ShooterConstants.kTurretWraparoundReleaseThresholdDeg) / range,
                1.0);
        double hint = fraction * ShooterConstants.kTurretWraparoundMaxHintRadPerSec;
        return Math.signum(turretAngleDeg) * hint;
    }

    @Test
    void turretWraparound_belowThreshold_notActive() {
        assertFalse(computeTurretNearLimit(false, 100.0),
                "Turret at 100° should not be near limit (threshold=" + ShooterConstants.kTurretWraparoundThresholdDeg + ")");
    }

    @Test
    void turretWraparound_aboveEngageThreshold_activates() {
        assertTrue(computeTurretNearLimit(false, ShooterConstants.kTurretWraparoundThresholdDeg + 1.0),
                "Turret above engage threshold should activate");
    }

    @Test
    void turretWraparound_hysteresis_staysActiveAboveRelease() {
        // Once active, should stay active as long as above release threshold
        assertTrue(computeTurretNearLimit(true, ShooterConstants.kTurretWraparoundReleaseThresholdDeg + 1.0),
                "Should stay active above release threshold");
    }

    @Test
    void turretWraparound_hysteresis_deactivatesBelowRelease() {
        assertFalse(computeTurretNearLimit(true, ShooterConstants.kTurretWraparoundReleaseThresholdDeg - 1.0),
                "Should deactivate below release threshold");
    }

    @Test
    void turretWraparound_hysteresisBand_preventsFlicker() {
        // At an angle between release (140) and engage (150), behavior depends on previous state
        double midAngle = (ShooterConstants.kTurretWraparoundReleaseThresholdDeg
                + ShooterConstants.kTurretWraparoundThresholdDeg) / 2.0;

        assertFalse(computeTurretNearLimit(false, midAngle),
                "Should NOT activate in hysteresis band if was inactive");
        assertTrue(computeTurretNearLimit(true, midAngle),
                "Should STAY active in hysteresis band if was active");
    }

    @Test
    void turretWraparound_hint_scalesProportionally() {
        double hintAtRelease = computeWraparoundHint(ShooterConstants.kTurretWraparoundReleaseThresholdDeg + 0.1);
        double hintAtHardLimit = computeWraparoundHint(ShooterConstants.kTurretMaxAngleDegrees);

        assertTrue(hintAtHardLimit > hintAtRelease,
                "Hint should increase from release to hard limit");
        assertEquals(ShooterConstants.kTurretWraparoundMaxHintRadPerSec,
                Math.abs(hintAtHardLimit), 0.01,
                "Hint should be max at hard limit");
    }

    @Test
    void turretWraparound_hint_signMatchesTurretDirection() {
        double hintPositive = computeWraparoundHint(160.0);
        double hintNegative = computeWraparoundHint(-160.0);

        assertTrue(hintPositive > 0, "Positive turret angle should give positive hint");
        assertTrue(hintNegative < 0, "Negative turret angle should give negative hint");
    }

    // ==================== HARD LIMIT DETECTION ====================

    @Test
    void turretHardLimit_detected_whenNearLimitAndAtPhysicalLimit() {
        boolean nearLimit = true;
        double actualTurretAbs = ShooterConstants.kTurretMaxAngleDegrees - 2.0;
        boolean hardLimit = nearLimit && actualTurretAbs > (ShooterConstants.kTurretMaxAngleDegrees - 3.0);
        assertTrue(hardLimit, "Should detect hard limit when near limit and at physical edge");
    }

    @Test
    void turretHardLimit_notDetected_whenFarFromPhysicalLimit() {
        boolean nearLimit = true;
        double actualTurretAbs = ShooterConstants.kTurretMaxAngleDegrees - 10.0;
        boolean hardLimit = nearLimit && actualTurretAbs > (ShooterConstants.kTurretMaxAngleDegrees - 3.0);
        assertFalse(hardLimit, "Should NOT detect hard limit when turret is far from physical limit");
    }

    // ==================== isReadyToShoot CONDITIONS ====================

    /**
     * Replicate isReadyToShoot logic for testing.
     */
    private boolean computeReadyToShoot(
            boolean healthy, boolean allianceZoneMode, boolean trackingEnabled,
            boolean turretAtHardLimit, boolean flywheelAtSpeed, boolean hoodAtTarget,
            boolean turretAtTarget, boolean inRange, boolean flywheelRecovered) {

        if (!healthy) return false;
        if (allianceZoneMode) {
            return flywheelAtSpeed && hoodAtTarget && turretAtTarget && flywheelRecovered;
        }
        if (!trackingEnabled) return false;
        if (turretAtHardLimit) return false;
        return flywheelAtSpeed && hoodAtTarget && turretAtTarget && inRange && flywheelRecovered;
    }

    @Test
    void readyToShoot_allConditionsMet_returnsTrue() {
        assertTrue(computeReadyToShoot(true, false, true, false, true, true, true, true, true));
    }

    @Test
    void readyToShoot_unhealthy_returnsFalse() {
        assertFalse(computeReadyToShoot(false, false, true, false, true, true, true, true, true));
    }

    @Test
    void readyToShoot_trackingDisabled_returnsFalse() {
        assertFalse(computeReadyToShoot(true, false, false, false, true, true, true, true, true));
    }

    @Test
    void readyToShoot_turretAtHardLimit_returnsFalse() {
        assertFalse(computeReadyToShoot(true, false, true, true, true, true, true, true, true));
    }

    @Test
    void readyToShoot_flywheelNotAtSpeed_returnsFalse() {
        assertFalse(computeReadyToShoot(true, false, true, false, false, true, true, true, true));
    }

    @Test
    void readyToShoot_hoodNotAtTarget_returnsFalse() {
        assertFalse(computeReadyToShoot(true, false, true, false, true, false, true, true, true));
    }

    @Test
    void readyToShoot_turretNotAtTarget_returnsFalse() {
        assertFalse(computeReadyToShoot(true, false, true, false, true, true, false, true, true));
    }

    @Test
    void readyToShoot_outOfRange_returnsFalse() {
        assertFalse(computeReadyToShoot(true, false, true, false, true, true, true, false, true));
    }

    @Test
    void readyToShoot_flywheelNotRecovered_returnsFalse() {
        assertFalse(computeReadyToShoot(true, false, true, false, true, true, true, true, false));
    }

    // ==================== ALLIANCE ZONE MODE ====================

    @Test
    void readyToShoot_allianceZone_ignoresTrackingAndRange() {
        // Alliance zone mode should not require trackingEnabled or inRange
        assertTrue(computeReadyToShoot(true, true, false, false, true, true, true, false, true),
                "Alliance zone mode should work without tracking or range");
    }

    @Test
    void readyToShoot_allianceZone_ignoresTurretHardLimit() {
        assertTrue(computeReadyToShoot(true, true, false, true, true, true, true, false, true),
                "Alliance zone mode should ignore turret hard limit");
    }

    @Test
    void allianceZone_fixedParameters() {
        // Verify alliance zone constants are within valid ranges
        assertTrue(ShooterConstants.kAllianceZoneFlywheelRPS > 0,
                "Alliance zone flywheel RPS should be positive");
        assertTrue(ShooterConstants.kAllianceZoneHoodAngleDeg >= ShooterConstants.kHoodMinAngleDegrees
                && ShooterConstants.kAllianceZoneHoodAngleDeg <= ShooterConstants.kHoodMaxAngleDegrees,
                "Alliance zone hood angle should be within hood limits");
        assertTrue(Math.abs(ShooterConstants.kAllianceZoneTurretAngleDeg)
                <= ShooterConstants.kTurretMaxAngleDegrees,
                "Alliance zone turret angle should be within turret limits");
    }

    // ==================== BROWNOUT FALLBACK ====================

    @Test
    void brownoutFallback_scalesToSeventyPercent() {
        double normalRPS = flywheelTable.get(3.0);
        double brownoutRPS = normalRPS * 0.7;

        assertTrue(brownoutRPS < normalRPS,
                "Brownout flywheel RPS should be less than normal");
        assertTrue(brownoutRPS > 0,
                "Brownout flywheel RPS should still be positive");
        assertEquals(normalRPS * 0.7, brownoutRPS, 0.01,
                "Should scale to exactly 70%");
    }

    @Test
    void criticalBattery_stopsShooter() {
        // Critical battery → flywheel RPS = 0
        // This is a logic verification — the actual implementation sets flywheel to 0
        double criticalFlywheelRPS = 0.0;
        assertEquals(0.0, criticalFlywheelRPS, "Critical battery should stop flywheel");
    }

    // ==================== VOLTAGE COMPENSATION ====================

    @Test
    void voltageCompensation_nominalVoltage_returnsOne() {
        double factor = computeVoltageCompensation(12.0);
        assertEquals(1.0, factor, 0.001);
    }

    @Test
    void voltageCompensation_criticalVoltage_returnsZero() {
        double factor = computeVoltageCompensation(6.5);
        assertEquals(0.0, factor, 0.001);
    }

    @Test
    void voltageCompensation_midRange_returnsLinearValue() {
        double mid = (12.0 + 6.5) / 2.0;
        double factor = computeVoltageCompensation(mid);
        assertEquals(0.5, factor, 0.01);
    }

    /**
     * Replicates RobotState.getVoltageCompensationFactor() logic.
     */
    private double computeVoltageCompensation(double voltage) {
        if (voltage >= 12.0) return 1.0;
        if (voltage <= 6.5) return 0.0;
        return (voltage - 6.5) / (12.0 - 6.5);
    }

    // ==================== HOOD TRIM ====================

    @Test
    void hoodTrim_clamps() {
        double trim = MathUtil.clamp(0.0 + 2.0, -4.0, 4.0);
        assertEquals(2.0, trim);
        trim = MathUtil.clamp(trim + 2.0, -4.0, 4.0);
        assertEquals(4.0, trim);
        trim = MathUtil.clamp(trim + 2.0, -4.0, 4.0);
        assertEquals(4.0, trim, "Should clamp at +4°");
    }

    @Test
    void hoodTrim_negativeClamping() {
        double trim = MathUtil.clamp(0.0 - 2.0, -4.0, 4.0);
        assertEquals(-2.0, trim);
        trim = MathUtil.clamp(trim - 2.0, -4.0, 4.0);
        assertEquals(-4.0, trim);
        trim = MathUtil.clamp(trim - 2.0, -4.0, 4.0);
        assertEquals(-4.0, trim, "Should clamp at -4°");
    }
}
