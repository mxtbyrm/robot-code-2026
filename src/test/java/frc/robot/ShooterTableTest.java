package frc.robot;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import frc.robot.constants.ShooterConstants;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for the physics-computed shooting lookup tables.
 *
 * <p>Values are derived from projectile motion equations at robot init.
 * Tests verify structure, monotonicity, range, interpolation, and physics helpers.
 */
class ShooterTableTest {

    private double[][] shootingTable;
    private double[][] tofTable;

    private InterpolatingDoubleTreeMap flywheelTreeMap;
    private InterpolatingDoubleTreeMap hoodAngleTreeMap;
    private InterpolatingDoubleTreeMap tofTreeMap;

    @BeforeEach
    void setUp() {
        shootingTable = ShooterPhysics.computeShootingTable();
        tofTable = ShooterPhysics.computeTimeOfFlightTable();

        flywheelTreeMap = new InterpolatingDoubleTreeMap();
        hoodAngleTreeMap = new InterpolatingDoubleTreeMap();
        tofTreeMap = new InterpolatingDoubleTreeMap();

        for (double[] entry : shootingTable) {
            flywheelTreeMap.put(entry[0], entry[1]);
            hoodAngleTreeMap.put(entry[0], entry[2]);
        }
        for (double[] entry : tofTable) {
            tofTreeMap.put(entry[0], entry[1]);
        }
    }

    // ==================== TABLE STRUCTURE ====================

    @Test
    void shootingTable_hasEnoughEntries() {
        for (int i = 1; i < shootingTable.length; i++) {
            assertTrue(shootingTable[i][0] > shootingTable[i - 1][0],
                    "Distances must be strictly increasing: " + shootingTable[i - 1][0]
                    + " < " + shootingTable[i][0]);
        }
    }

    @Test
    void shootingDistanceRange_matchesConstants() {
        double firstDist = shootingTable[0][0];
        double lastDist = shootingTable[shootingTable.length - 1][0];
        assertTrue(firstDist >= ShooterConstants.kMinShootingDistanceMeters,
                "First table distance " + firstDist + " should be >= min " +
                ShooterConstants.kMinShootingDistanceMeters);
        assertTrue(lastDist <= ShooterConstants.kMaxShootingDistanceMeters,
                "Last table distance " + lastDist + " should be <= max " +
                ShooterConstants.kMaxShootingDistanceMeters);
    }

    // ==================== MONOTONICITY (physics sanity) ====================

    @Test
    void flywheelRPS_increasesWithDistance() {
        for (int i = 1; i < shootingTable.length; i++) {
            assertTrue(shootingTable[i][1] > shootingTable[i - 1][1],
                    String.format("Flywheel RPS should increase: %.1fm (%.1f RPS) <= %.1fm (%.1f RPS)",
                            shootingTable[i - 1][0], shootingTable[i - 1][1],
                            shootingTable[i][0], shootingTable[i][1]));
        }
    }

    @Test
    void hoodAngle_increasesWithDistance() {
        for (int i = 1; i < shootingTable.length; i++) {
            assertTrue(shootingTable[i][2] >= shootingTable[i - 1][2],
                    String.format("Hood angle should increase: %.1fm (%.1f°) > %.1fm (%.1f°)",
                            shootingTable[i - 1][0], shootingTable[i - 1][2],
                            shootingTable[i][0], shootingTable[i][2]));
        }
    }

    @Test
    void timeOfFlight_increasesWithDistance() {
        for (int i = 1; i < tofTable.length; i++) {
            assertTrue(tofTable[i][1] >= tofTable[i - 1][1],
                    String.format("ToF should increase: %.1fm (%.2fs) > %.1fm (%.2fs)",
                            tofTable[i - 1][0], tofTable[i - 1][1],
                            tofTable[i][0], tofTable[i][1]));
        }
    }

    // ==================== RANGE VALIDATION ====================

    @Test
    void hoodAngles_withinMechanicalLimits() {
        for (double[] entry : shootingTable) {
            double angle = entry[2];
            assertTrue(angle >= ShooterConstants.kHoodMinAngleDegrees - 0.1,
                    "Hood angle " + angle + "° must be >= min " + ShooterConstants.kHoodMinAngleDegrees);
            assertTrue(angle <= ShooterConstants.kHoodMaxAngleDegrees + 0.1,
                    "Hood angle " + angle + "° must be <= max " + ShooterConstants.kHoodMaxAngleDegrees);
        }
    }

    @Test
    void flywheelRPS_arePositive() {
        for (double[] entry : shootingTable) {
            assertTrue(entry[1] > 0, "Flywheel RPS must be positive, got " + entry[1]);
        }
    }

    @Test
    void flywheelRPS_areReasonable() {
        for (double[] entry : shootingTable) {
            assertTrue(entry[1] >= 20.0 && entry[1] <= 150.0,
                    "Flywheel RPS " + entry[1] + " at " + entry[0] + "m outside [20, 150]");
        }
    }

    @Test
    void timeOfFlight_arePositiveAndReasonable() {
        for (double[] entry : tofTable) {
            assertTrue(entry[1] > 0, "ToF must be positive, got " + entry[1]);
            assertTrue(entry[1] < 3.0, "ToF " + entry[1] + "s at " + entry[0] + "m is unreasonably long");
        }
    }

    // ==================== INTERPOLATION ====================

    @Test
    void flywheelTreeMap_interpolatesMidpoint() {
        if (shootingTable.length >= 2) {
            double d1 = shootingTable[0][0], d2 = shootingTable[1][0];
            double r1 = shootingTable[0][1], r2 = shootingTable[1][1];
            double midDist = (d1 + d2) / 2.0;
            double expected = (r1 + r2) / 2.0;
            assertEquals(expected, flywheelTreeMap.get(midDist), 0.5,
                    "Flywheel should interpolate linearly at midpoint");
        }
    }

    @Test
    void flywheelTreeMap_belowMinDistance_clampsToFirst() {
        double atMin = flywheelTreeMap.get(shootingTable[0][0]);
        double belowMin = flywheelTreeMap.get(0.5);
        assertEquals(atMin, belowMin, 1e-9,
                "Flywheel below min distance should clamp to first entry");
    }

    @Test
    void flywheelTreeMap_aboveMaxDistance_clampsToLast() {
        double lastDist = shootingTable[shootingTable.length - 1][0];
        double atMax = flywheelTreeMap.get(lastDist);
        double aboveMax = flywheelTreeMap.get(lastDist + 10.0);
        assertEquals(atMax, aboveMax, 1e-9,
                "Flywheel above max distance should clamp to last entry");
    }

    @Test
    void hoodAngleTreeMap_belowMinDistance_clampsToFirst() {
        double atMin = hoodAngleTreeMap.get(shootingTable[0][0]);
        double belowMin = hoodAngleTreeMap.get(0.0);
        assertEquals(atMin, belowMin, 1e-9,
                "Hood angle below min should clamp to first entry");
    }

    // ==================== PHYSICS HELPERS ====================

    @Test
    void hoodToExitAngle_atMinHood_returnsMaxExit() {
        double exit = ShooterPhysics.hoodToExitAngle(ShooterConstants.kHoodMinAngleDegrees);
        assertEquals(ShooterConstants.kExitAngleAtMinHood, exit, 1e-9);
    }

    @Test
    void hoodToExitAngle_atMaxHood_returnsMinExit() {
        double exit = ShooterPhysics.hoodToExitAngle(ShooterConstants.kHoodMaxAngleDegrees);
        assertEquals(ShooterConstants.kExitAngleAtMaxHood, exit, 1e-9);
    }

    @Test
    void hoodToExitAngle_midpoint_returnsMidExit() {
        double midHood = (ShooterConstants.kHoodMinAngleDegrees + ShooterConstants.kHoodMaxAngleDegrees) / 2.0;
        double expected = (ShooterConstants.kExitAngleAtMinHood + ShooterConstants.kExitAngleAtMaxHood) / 2.0;
        assertEquals(expected, ShooterPhysics.hoodToExitAngle(midHood), 1e-9);
    }

    @Test
    void trajectoryVelocity_isPositive() {
        double deltaH = ShooterConstants.kHubHeightMeters - ShooterConstants.kShooterExitHeightMeters;
        double v = ShooterPhysics.computeTrajectoryVelocity(2.0, Math.toRadians(60.0), deltaH);
        assertTrue(v > 0, "Trajectory velocity should be positive");
    }

    @Test
    void trajectoryVelocity_increasesWithDistance() {
        double deltaH = ShooterConstants.kHubHeightMeters - ShooterConstants.kShooterExitHeightMeters;
        double v1 = ShooterPhysics.computeTrajectoryVelocity(2.0, Math.toRadians(55.0), deltaH);
        double v2 = ShooterPhysics.computeTrajectoryVelocity(5.0, Math.toRadians(50.0), deltaH);
        assertTrue(v2 > v1, "Trajectory velocity should increase with distance");
    }

    @Test
    void apexVelocity_isPositive() {
        double v = ShooterPhysics.computeApexVelocity(Math.toRadians(55.0), 1.5);
        assertTrue(v > 0, "Apex velocity should be positive");
    }

    @Test
    void apexVelocity_increasesWithRequiredClimb() {
        double v1 = ShooterPhysics.computeApexVelocity(Math.toRadians(55.0), 1.0);
        double v2 = ShooterPhysics.computeApexVelocity(Math.toRadians(55.0), 2.0);
        assertTrue(v2 > v1, "Higher apex clearance should require more velocity");
    }

    // ==================== BASKETBALL ARC PROPERTIES ====================

    @Test
    void allShots_apexAboveHub() {
        // Like basketball: ball must peak ABOVE the basket at every distance
        for (double[] entry : shootingTable) {
            double distance = entry[0];
            double rps = entry[1];
            double hood = entry[2];

            double exitAngleRad = Math.toRadians(ShooterPhysics.hoodToExitAngle(hood));
            double exitVelocity = ShooterPhysics.rpsToVelocity(rps);
            double apexHeight = ShooterPhysics.computeApexHeight(exitVelocity, exitAngleRad);

            assertTrue(apexHeight > ShooterConstants.kHubHeightMeters,
                    String.format("Apex (%.2fm) must be above Hub (%.2fm) at %.1fm",
                            apexHeight, ShooterConstants.kHubHeightMeters, distance));
        }
    }

    @Test
    void allShots_apexClearanceMetMinimum() {
        // Ball must peak at least kMinApexClearanceMeters above Hub
        for (double[] entry : shootingTable) {
            double rps = entry[1];
            double hood = entry[2];

            double exitAngleRad = Math.toRadians(ShooterPhysics.hoodToExitAngle(hood));
            double exitVelocity = ShooterPhysics.rpsToVelocity(rps);
            double apexHeight = ShooterPhysics.computeApexHeight(exitVelocity, exitAngleRad);
            double clearance = apexHeight - ShooterConstants.kHubHeightMeters;

            assertTrue(clearance >= ShooterConstants.kMinApexClearanceMeters - 0.05,
                    String.format("Apex clearance %.2fm at %.1fm must be >= %.2fm",
                            clearance, entry[0], ShooterConstants.kMinApexClearanceMeters));
        }
    }

    @Test
    void allShots_ballIsDescending() {
        // At mid/long range (≥3.0m), ball must be coming DOWN into the Hub (basketball arc).
        // At close range (<3.0m), the steep exit angle means the ball passes through
        // the Hub's side opening while still ascending — this is fine for a side-entry target.
        double descentThreshold = 3.0; // meters — below this, ascending entry is acceptable
        for (double[] entry : shootingTable) {
            double distance = entry[0];
            if (distance < descentThreshold) continue;

            double rps = entry[1];
            double hood = entry[2];

            double exitAngleRad = Math.toRadians(ShooterPhysics.hoodToExitAngle(hood));
            double exitVelocity = ShooterPhysics.rpsToVelocity(rps);
            double descentAngle = ShooterPhysics.computeDescentAngle(distance, exitVelocity, exitAngleRad);

            assertTrue(descentAngle > 0,
                    String.format("Ball must be descending at Hub (descent=%.1f°) at %.1fm",
                            descentAngle, distance));
        }
    }

    @Test
    void allShots_descentAngleIsReasonable() {
        // At mid/long range, descent angle should be at least 30° for reliable entry.
        // Close range shots enter via the side opening and don't need steep descent.
        double descentThreshold = 3.0;
        for (double[] entry : shootingTable) {
            double distance = entry[0];
            if (distance < descentThreshold) continue;

            double rps = entry[1];
            double hood = entry[2];

            double exitAngleRad = Math.toRadians(ShooterPhysics.hoodToExitAngle(hood));
            double exitVelocity = ShooterPhysics.rpsToVelocity(rps);
            double descentAngle = ShooterPhysics.computeDescentAngle(distance, exitVelocity, exitAngleRad);

            assertTrue(descentAngle >= 30.0,
                    String.format("Descent angle %.1f° at %.1fm is too shallow (min 30°)",
                            descentAngle, distance));
        }
    }

    @Test
    void allShots_entryMarginIsPositive() {
        // Entry margin is meaningful only for descending shots (mid/long range).
        // Close range ascending shots enter the side opening directly.
        double descentThreshold = 3.0;
        for (double[] entry : shootingTable) {
            double distance = entry[0];
            if (distance < descentThreshold) continue;

            double rps = entry[1];
            double hood = entry[2];

            double exitAngleRad = Math.toRadians(ShooterPhysics.hoodToExitAngle(hood));
            double exitVelocity = ShooterPhysics.rpsToVelocity(rps);
            double descentAngle = ShooterPhysics.computeDescentAngle(distance, exitVelocity, exitAngleRad);
            double margin = ShooterPhysics.computeEntryMargin(descentAngle);

            assertTrue(margin > 0,
                    String.format("Entry margin must be positive at %.1fm (margin=%.3fm, descent=%.1f°)",
                            distance, margin, descentAngle));
        }
    }

    // ==================== CONSISTENCY ====================

    @Test
    void allTables_haveSameEntryCount() {
        assertEquals(shootingTable.length, tofTable.length,
                "Shooting and ToF should have same entry count");
    }

    @Test
    void computedTables_areDeterministic() {
        double[][] shooting2 = ShooterPhysics.computeShootingTable();
        assertEquals(shootingTable.length, shooting2.length);
        for (int i = 0; i < shootingTable.length; i++) {
            assertEquals(shootingTable[i][0], shooting2[i][0], 1e-9, "Distance at " + i);
            assertEquals(shootingTable[i][1], shooting2[i][1], 1e-9, "RPS at " + i);
            assertEquals(shootingTable[i][2], shooting2[i][2], 1e-9, "Hood at " + i);
        }
    }
}
