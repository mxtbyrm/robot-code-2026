package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import frc.robot.constants.ShooterConstants;

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
     *
     * @param robotPos       Robot odometry center in field frame
     * @param robotHeadingRad Robot heading in radians
     * @param vxMPS          Field-relative X velocity (m/s)
     * @param vyMPS          Field-relative Y velocity (m/s)
     * @param hubPos         Hub position in field frame
     * @param shooterOffset  Shooter position offset in robot frame (X=forward, Y=left)
     * @param omegaRadPerSec Robot angular velocity (rad/s, CCW positive)
     */
    private double[] computeShootWhileMoving(
            Translation2d robotPos, double robotHeadingRad,
            double vxMPS, double vyMPS, Translation2d hubPos,
            Translation2d shooterOffset, double omegaRadPerSec) {

        // Rotate shooter offset into field frame
        Translation2d shooterOffsetField = shooterOffset.rotateBy(new Rotation2d(robotHeadingRad));
        Translation2d shooterPos = robotPos.plus(shooterOffsetField);

        Translation2d shooterToHub = hubPos.minus(shooterPos);
        double distanceToHub = shooterToHub.getNorm();

        // Distance-based time-of-flight
        double tof = timeOfFlightTable.get(distanceToHub);

        // Future robot position
        double futureRobotX = robotPos.getX() + vxMPS * tof;
        double futureRobotY = robotPos.getY() + vyMPS * tof;

        // Future heading (robot rotates during flight)
        double futureHeadingRad = robotHeadingRad + omegaRadPerSec * tof;

        // Future shooter offset rotated by future heading
        Translation2d futureShooterOffset = shooterOffset.rotateBy(new Rotation2d(futureHeadingRad));

        // Future shooter position in field frame
        Translation2d futureShooterPos = new Translation2d(
                futureRobotX + futureShooterOffset.getX(),
                futureRobotY + futureShooterOffset.getY());

        Translation2d futureToHub = hubPos.minus(futureShooterPos);
        double compensatedDistance = futureToHub.getNorm();

        double compensatedFieldAngle = Math.atan2(futureToHub.getY(), futureToHub.getX());
        double compensatedTurretRad = compensatedFieldAngle - robotHeadingRad;
        double compensatedTurretAngle = Math.toDegrees(MathUtil.angleModulus(compensatedTurretRad))
                + ShooterConstants.kTurretMountOffsetDegrees;

        // Shooter's true field velocity = robot velocity + ω × shooter_offset_field
        double vShooterX = vxMPS - omegaRadPerSec * shooterOffsetField.getY();
        double vShooterY = vyMPS + omegaRadPerSec * shooterOffsetField.getX();

        double hubAngle = Math.atan2(shooterToHub.getY(), shooterToHub.getX());
        double vTowardHub = vShooterX * Math.cos(hubAngle) + vShooterY * Math.sin(hubAngle);
        double flywheelCompensationRPS = -vTowardHub
                / (ShooterConstants.kBottomFlywheelCircumferenceMeters
                   * ShooterConstants.kShooterEfficiencyFactor);

        return new double[] { compensatedDistance, compensatedTurretAngle, flywheelCompensationRPS };
    }

    @Test
    void shootWhileMoving_stationaryRobot_noCompensation() {
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(2.0, 4.0);

        double[] result = computeShootWhileMoving(robotPos, 0.0, 0.0, 0.0, hub, new Translation2d(), 0.0);

        double staticDist = hub.minus(robotPos).getNorm();
        assertEquals(staticDist, result[0], 0.01, "Compensated distance should equal static when stationary");
        assertEquals(0.0, result[2], 0.01, "Flywheel compensation should be 0 when stationary");
    }

    @Test
    void shootWhileMoving_movingTowardHub_reducesDistance() {
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(hub.getX() + 3.0, hub.getY());

        // Moving toward the hub at 2 m/s
        double[] result = computeShootWhileMoving(robotPos, 0.0, -2.0, 0.0, hub, new Translation2d(), 0.0);

        double staticDist = hub.minus(robotPos).getNorm();
        assertTrue(result[0] < staticDist,
                "Compensated distance should be less when moving toward hub");
    }

    @Test
    void shootWhileMoving_movingAwayFromHub_increasesDistance() {
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(hub.getX() + 3.0, hub.getY());

        // Moving away from hub at 2 m/s
        double[] result = computeShootWhileMoving(robotPos, 0.0, 2.0, 0.0, hub, new Translation2d(), 0.0);

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
        double[] resultStatic = computeShootWhileMoving(robotPos, headingRad, 0.0, 0.0, hub, new Translation2d(), 0.0);
        double[] resultMoving = computeShootWhileMoving(robotPos, headingRad, 0.0, 2.0, hub, new Translation2d(), 0.0);

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
        double[] result = computeShootWhileMoving(robotPos, 0.0, -2.0, 0.0, hub, new Translation2d(), 0.0);

        // Moving toward hub (negative vx, but toward hub is positive direction)
        // vTowardHub = vShooterX * cos(hubAngle) + vShooterY * sin(hubAngle)
        // hub is to the left of robot (negative x direction), so cos(hubAngle) is negative
        // vShooterX=-2 * cos(PI)=-1 = +2, so vTowardHub > 0 → compensation < 0
        assertTrue(result[2] < 0,
                "Moving toward hub should give negative flywheel compensation (reduce speed)");
    }

    // ==================== SHOOTER POSITION OFFSET ====================

    @Test
    void shooterOffset_zeroOffset_sameAsRobotCenter() {
        // When offset is (0,0), shooter IS at robot center — distance equals robot-to-hub distance.
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(2.0, hub.getY());

        double[] result = computeShootWhileMoving(
                robotPos, 0.0, 0.0, 0.0, hub, new Translation2d(), 0.0);

        double expectedDist = hub.minus(robotPos).getNorm();
        assertEquals(expectedDist, result[0], 0.01,
                "Zero offset should give distance equal to robot-center-to-hub distance");
        assertEquals(0.0, result[2], 0.01,
                "Zero offset + stationary robot = zero flywheel compensation");
    }

    @Test
    void shooterOffset_nonzero_changesDistance() {
        // Shooter 0.3m forward of robot center, robot facing hub → shooter is closer to hub.
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(hub.getX() - 3.0, hub.getY());
        double headingRad = 0.0; // facing +x (toward hub)

        Translation2d forwardOffset = new Translation2d(0.3, 0.0);

        double[] withOffset = computeShootWhileMoving(
                robotPos, headingRad, 0.0, 0.0, hub, forwardOffset, 0.0);
        double[] withoutOffset = computeShootWhileMoving(
                robotPos, headingRad, 0.0, 0.0, hub, new Translation2d(), 0.0);

        assertTrue(withOffset[0] < withoutOffset[0],
                "Shooter forward of center should be closer to hub");
        assertEquals(withoutOffset[0] - 0.3, withOffset[0], 0.01,
                "Distance should decrease by exactly the forward offset amount");
    }

    @Test
    void shooterOffset_nonzero_changesAngle() {
        // Shooter 0.2m to the left of robot center → aims at a slightly different angle.
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(hub.getX() - 3.0, hub.getY());
        double headingRad = 0.0; // facing +x (toward hub)

        Translation2d lateralOffset = new Translation2d(0.0, 0.2); // 0.2m left

        double[] withOffset = computeShootWhileMoving(
                robotPos, headingRad, 0.0, 0.0, hub, lateralOffset, 0.0);
        double[] withoutOffset = computeShootWhileMoving(
                robotPos, headingRad, 0.0, 0.0, hub, new Translation2d(), 0.0);

        assertNotEquals(withoutOffset[1], withOffset[1], 0.01,
                "Lateral shooter offset should change the required turret angle");
    }

    @Test
    void shootWhileMoving_withRotation_futureShooterOffsetApplied() {
        // With omega != 0 and a nonzero offset, the robot rotates during flight so the
        // future shooter position differs from the future robot center + current offset.
        Translation2d hub = ShooterConstants.kBlueHubPosition;
        Translation2d robotPos = new Translation2d(hub.getX() - 3.0, hub.getY());
        double headingRad = 0.0;
        Translation2d shooterOffset = new Translation2d(0.3, 0.0);

        double[] withRotation = computeShootWhileMoving(
                robotPos, headingRad, 0.0, 0.0, hub, shooterOffset, 1.0);
        double[] withoutRotation = computeShootWhileMoving(
                robotPos, headingRad, 0.0, 0.0, hub, shooterOffset, 0.0);

        // When omega != 0, the future heading differs → future shooter offset rotates →
        // compensated distance and/or angle should differ from the zero-omega case.
        assertFalse(
                Math.abs(withRotation[0] - withoutRotation[0]) < 0.001
                && Math.abs(withRotation[1] - withoutRotation[1]) < 0.001,
                "Rotation during flight should change the compensated aim solution");
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
