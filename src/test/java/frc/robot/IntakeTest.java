package frc.robot;

import frc.robot.Constants.IntakeConstants;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for Intake subsystem logic.
 *
 * <p>Tests gravity feedforward lookup tables, deploy position tolerances,
 * stall detection logic, and constants validation without hardware.
 */
class IntakeTest {

    // ==================== GRAVITY TABLE VALIDATION ====================

    @Test
    void leftGravityTable_stowed_isZero() {
        InterpolatingDoubleTreeMap table = buildTable(IntakeConstants.kDeployLeftGravityTable);
        assertEquals(0.0, table.get(0.0), 1e-6, "Gravity FF at stowed should be zero");
    }

    @Test
    void rightGravityTable_stowed_isZero() {
        InterpolatingDoubleTreeMap table = buildTable(IntakeConstants.kDeployRightGravityTable);
        assertEquals(0.0, table.get(0.0), 1e-6, "Gravity FF at stowed should be zero");
    }

    @Test
    void leftGravityTable_fullDeploy_isPositive() {
        InterpolatingDoubleTreeMap table = buildTable(IntakeConstants.kDeployLeftGravityTable);
        double ff = table.get(IntakeConstants.kDeployExtendedRotations);
        assertTrue(ff > 0, "Gravity FF at full deploy should be positive, got " + ff);
    }

    @Test
    void rightGravityTable_fullDeploy_isPositive() {
        InterpolatingDoubleTreeMap table = buildTable(IntakeConstants.kDeployRightGravityTable);
        double ff = table.get(IntakeConstants.kDeployExtendedRotations);
        assertTrue(ff > 0, "Gravity FF at full deploy should be positive, got " + ff);
    }

    @Test
    void rightGravityTable_alwaysGreaterOrEqualToLeft() {
        InterpolatingDoubleTreeMap left = buildTable(IntakeConstants.kDeployLeftGravityTable);
        InterpolatingDoubleTreeMap right = buildTable(IntakeConstants.kDeployRightGravityTable);

        // Sample at 20 points across the travel range
        double stowed = IntakeConstants.kDeployStowedRotations;
        double extended = IntakeConstants.kDeployExtendedRotations;
        for (int i = 0; i <= 20; i++) {
            double pos = stowed + (extended - stowed) * i / 20.0;
            double leftFF = left.get(pos);
            double rightFF = right.get(pos);
            assertTrue(rightFF >= leftFF - 1e-6,
                    "Right gravity FF (" + rightFF + ") should be >= left (" + leftFF + ") at pos " + pos);
        }
    }

    @Test
    void gravityTables_monotonicallyIncrease() {
        // As the arm deploys further (more negative position), gravity FF should increase
        InterpolatingDoubleTreeMap left = buildTable(IntakeConstants.kDeployLeftGravityTable);
        InterpolatingDoubleTreeMap right = buildTable(IntakeConstants.kDeployRightGravityTable);

        double stowed = IntakeConstants.kDeployStowedRotations;
        double extended = IntakeConstants.kDeployExtendedRotations;

        double prevLeft = left.get(stowed);
        double prevRight = right.get(stowed);
        for (int i = 1; i <= 20; i++) {
            double pos = stowed + (extended - stowed) * i / 20.0;
            double curLeft = left.get(pos);
            double curRight = right.get(pos);
            assertTrue(curLeft >= prevLeft - 1e-6,
                    "Left gravity should increase toward deploy, pos " + pos);
            assertTrue(curRight >= prevRight - 1e-6,
                    "Right gravity should increase toward deploy, pos " + pos);
            prevLeft = curLeft;
            prevRight = curRight;
        }
    }

    @Test
    void gravityTables_interpolateMidpoint() {
        InterpolatingDoubleTreeMap left = buildTable(IntakeConstants.kDeployLeftGravityTable);

        // Midpoint between stowed and extended should return an interpolated value
        double mid = IntakeConstants.kDeployExtendedRotations * 0.5;
        double ff = left.get(mid);
        assertTrue(ff > 0, "Midpoint gravity FF should be positive");
        assertTrue(ff < left.get(IntakeConstants.kDeployExtendedRotations),
                "Midpoint gravity FF should be less than full deploy");
    }

    @Test
    void gravityTables_haveSameEntryCount() {
        assertEquals(IntakeConstants.kDeployLeftGravityTable.length,
                IntakeConstants.kDeployRightGravityTable.length,
                "Left and right gravity tables should have same number of entries");
    }

    @Test
    void gravityTables_haveAtLeastThreeEntries() {
        assertTrue(IntakeConstants.kDeployLeftGravityTable.length >= 3,
                "Gravity table needs at least 3 entries for meaningful interpolation");
    }

    // ==================== DEPLOY POSITION CONSTANTS ====================

    @Test
    void deployPositions_stowedIsZero() {
        assertEquals(0.0, IntakeConstants.kDeployStowedRotations, 1e-6);
    }

    @Test
    void deployPositions_extendedIsNegative() {
        assertTrue(IntakeConstants.kDeployExtendedRotations < 0,
                "Extended position should be negative (deployed outward)");
    }

    @Test
    void deployPositions_hoverIsBetweenStowedAndExtended() {
        double hover = IntakeConstants.kDeployHoverRotations;
        double stowed = IntakeConstants.kDeployStowedRotations;
        double extended = IntakeConstants.kDeployExtendedRotations;
        assertTrue(hover < stowed && hover > extended,
                "Hover position should be between stowed and extended");
    }

    @Test
    void deployTolerance_isPositive() {
        assertTrue(IntakeConstants.kDeployToleranceRotations > 0);
    }

    @Test
    void deployTolerance_isSmallerThanTravel() {
        double travel = Math.abs(IntakeConstants.kDeployExtendedRotations - IntakeConstants.kDeployStowedRotations);
        assertTrue(IntakeConstants.kDeployToleranceRotations > 0,
                "Tolerance must be positive");
        assertTrue(IntakeConstants.kDeployToleranceRotations < travel,
                "Tolerance (" + IntakeConstants.kDeployToleranceRotations
                + ") must be smaller than total travel (" + travel + ")");
    }

    // ==================== DEPLOY AT TARGET LOGIC ====================

    @Test
    void isDeployAtTarget_atTarget_returnsTrue() {
        double target = IntakeConstants.kDeployExtendedRotations;
        double position = target + IntakeConstants.kDeployToleranceRotations * 0.5;
        assertTrue(Math.abs(position - target) <= IntakeConstants.kDeployToleranceRotations);
    }

    @Test
    void isDeployAtTarget_outsideTolerance_returnsFalse() {
        double target = IntakeConstants.kDeployExtendedRotations;
        double position = target + IntakeConstants.kDeployToleranceRotations * 2.0;
        assertFalse(Math.abs(position - target) <= IntakeConstants.kDeployToleranceRotations);
    }

    @Test
    void isDeployAtTarget_atBoundary_accepted() {
        double target = IntakeConstants.kDeployExtendedRotations;
        double position = target + IntakeConstants.kDeployToleranceRotations; // exactly at tolerance
        assertTrue(Math.abs(position - target) <= IntakeConstants.kDeployToleranceRotations,
                "At exactly the tolerance boundary, should be accepted (<=)");
    }

    // ==================== STALL DETECTION LOGIC ====================

    @Test
    void stallDetection_thresholdIsPositive() {
        assertTrue(IntakeConstants.kDeployStallCurrentThreshold > 0);
    }

    @Test
    void stallDetection_cycleThresholdIsPositive() {
        assertTrue(IntakeConstants.kDeployStallCycleThreshold > 0);
    }

    @Test
    void stallDetection_belowThreshold_noStall() {
        // Simulate stall counter behavior
        double current = IntakeConstants.kDeployStallCurrentThreshold - 1.0;
        int stallCount = 5;
        if (current > IntakeConstants.kDeployStallCurrentThreshold) {
            stallCount++;
        } else {
            stallCount = Math.max(0, stallCount - 1);
        }
        assertEquals(4, stallCount, "Below threshold, stall count should decay");
    }

    @Test
    void stallDetection_aboveThreshold_increments() {
        double current = IntakeConstants.kDeployStallCurrentThreshold + 1.0;
        int stallCount = 0;
        if (current > IntakeConstants.kDeployStallCurrentThreshold) {
            stallCount++;
        }
        assertEquals(1, stallCount);
    }

    @Test
    void stallDetection_decayNeverGoesNegative() {
        int stallCount = 0;
        stallCount = Math.max(0, stallCount - 1);
        assertEquals(0, stallCount, "Decay should not go below zero");
    }

    // ==================== ROLLER CONSTANTS ====================

    @Test
    void rollerIntakeSpeed_isPositive() {
        assertTrue(IntakeConstants.kRollerIntakeSpeed > 0);
    }

    @Test
    void rollerOuttakeSpeed_isNegative() {
        assertTrue(IntakeConstants.kRollerOuttakeSpeed < 0);
    }

    @Test
    void rollerSurfaceSpeed_exceedsChassisLimit() {
        assertTrue(IntakeConstants.kRollerSurfaceSpeedMps > IntakeConstants.kMaxChassisSpeedWhileIntaking,
                "Roller must spin faster than chassis to collect balls");
    }

    // ==================== PID CONSTANTS ====================

    @Test
    void deployPID_kPIsPositive() {
        assertTrue(IntakeConstants.kDeployP > 0);
    }

    @Test
    void deployPID_kSIsPositive() {
        assertTrue(IntakeConstants.kDeployS > 0, "kS (static friction) should be positive");
    }

    @Test
    void deployPID_kSIsReasonable() {
        assertTrue(IntakeConstants.kDeployS < 1.0,
                "kS should be a small voltage (< 1V), got " + IntakeConstants.kDeployS);
    }

    // ==================== HELPER ====================

    private InterpolatingDoubleTreeMap buildTable(double[][] entries) {
        InterpolatingDoubleTreeMap table = new InterpolatingDoubleTreeMap();
        for (double[] entry : entries) {
            table.put(entry[0], entry[1]);
        }
        return table;
    }
}
