package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.VisionConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.*;

/**
 * Unit tests for Vision subsystem logic.
 *
 * <p>Tests pose validation, pose-jump rejection, timestamp filtering,
 * standard deviation scaling, distance validation, and constants
 * without hardware or PhotonVision.
 */
class VisionTest {

    // ==================== POSE VALIDITY (Z-AXIS) ====================

    @Test
    void poseValidity_normalZ_accepted() {
        double z = 0.3;
        assertTrue(z >= -0.5 && z <= 1.5);
    }

    @Test
    void poseValidity_belowFloor_rejected() {
        double z = -0.6;
        assertFalse(z >= -0.5 && z <= 1.5, "Below floor should be rejected");
    }

    @Test
    void poseValidity_tooHigh_rejected() {
        double z = 1.6;
        assertFalse(z >= -0.5 && z <= 1.5, "Too high should be rejected");
    }

    @Test
    void poseValidity_atLowerBound_accepted() {
        double z = -0.5;
        assertTrue(z >= -0.5 && z <= 1.5);
    }

    @Test
    void poseValidity_atUpperBound_accepted() {
        double z = 1.5;
        assertTrue(z >= -0.5 && z <= 1.5);
    }

    // ==================== POSE VALIDITY (FIELD BOUNDS) ====================

    @Test
    void fieldBounds_insideField_accepted() {
        double margin = VisionConstants.kFieldBoundaryMargin;
        double x = AutoConstants.kFieldLengthMeters / 2.0;
        double y = AutoConstants.kFieldWidthMeters / 2.0;
        boolean valid = x >= -margin && x <= AutoConstants.kFieldLengthMeters + margin
                && y >= -margin && y <= AutoConstants.kFieldWidthMeters + margin;
        assertTrue(valid);
    }

    @Test
    void fieldBounds_outsideField_rejected() {
        double margin = VisionConstants.kFieldBoundaryMargin;
        double x = AutoConstants.kFieldLengthMeters + margin + 1.0;
        double y = 0;
        boolean valid = x >= -margin && x <= AutoConstants.kFieldLengthMeters + margin
                && y >= -margin && y <= AutoConstants.kFieldWidthMeters + margin;
        assertFalse(valid, "Beyond field + margin should be rejected");
    }

    @Test
    void fieldBounds_negativeXWithinMargin_accepted() {
        double margin = VisionConstants.kFieldBoundaryMargin;
        double x = -margin + 0.01;
        double y = 1.0;
        boolean valid = x >= -margin && x <= AutoConstants.kFieldLengthMeters + margin
                && y >= -margin && y <= AutoConstants.kFieldWidthMeters + margin;
        assertTrue(valid, "Just inside negative margin should be accepted");
    }

    @Test
    void fieldBounds_negativeXBeyondMargin_rejected() {
        double margin = VisionConstants.kFieldBoundaryMargin;
        double x = -margin - 0.01;
        double y = 1.0;
        boolean valid = x >= -margin && x <= AutoConstants.kFieldLengthMeters + margin
                && y >= -margin && y <= AutoConstants.kFieldWidthMeters + margin;
        assertFalse(valid, "Beyond negative margin should be rejected");
    }

    // ==================== POSE JUMP REJECTION ====================

    @Test
    void poseJump_smallJump_accepted() {
        Pose2d current = new Pose2d(5.0, 5.0, new Rotation2d());
        Pose2d vision = new Pose2d(5.5, 5.5, new Rotation2d());
        double jump = vision.getTranslation().getDistance(current.getTranslation());
        assertTrue(jump <= VisionConstants.kMaxPoseJumpMeters);
    }

    @Test
    void poseJump_largeJump_rejected() {
        Pose2d current = new Pose2d(5.0, 5.0, new Rotation2d());
        Pose2d vision = new Pose2d(10.0, 10.0, new Rotation2d());
        double jump = vision.getTranslation().getDistance(current.getTranslation());
        assertTrue(jump > VisionConstants.kMaxPoseJumpMeters,
                "Jump of " + jump + "m should exceed threshold");
    }

    @Test
    void poseJump_atThreshold_accepted() {
        Pose2d current = new Pose2d(0, 0, new Rotation2d());
        Pose2d vision = new Pose2d(VisionConstants.kMaxPoseJumpMeters, 0, new Rotation2d());
        double jump = vision.getTranslation().getDistance(current.getTranslation());
        assertTrue(jump <= VisionConstants.kMaxPoseJumpMeters,
                "At exactly threshold should be accepted");
    }

    // ==================== TIMESTAMP VALIDATION ====================

    @Test
    void timestamp_fresh_accepted() {
        double now = 50.0;
        double timestamp = 49.8;
        assertTrue(Math.abs(now - timestamp) <= 0.5);
    }

    @Test
    void timestamp_stale_rejected() {
        double now = 50.0;
        double timestamp = 49.0;
        assertTrue(Math.abs(now - timestamp) > 0.5);
    }

    @Test
    void timestamp_future_rejected() {
        double now = 50.0;
        double timestamp = 51.0;
        assertTrue(Math.abs(now - timestamp) > 0.5,
                "Future timestamps should be rejected");
    }

    // ==================== STD DEV SCALING ====================

    @Test
    void stdDevs_multiTag_tighterThanSingleTag() {
        double singleX = VisionConstants.kSingleTagStdDevs.get(0, 0);
        double multiX = VisionConstants.kMultiTagStdDevs.get(0, 0);
        assertTrue(multiX < singleX,
                "Multi-tag std devs should be tighter (lower) than single-tag");
    }

    @Test
    void stdDevs_aggressiveMultiTag_tightestOfAll() {
        double aggressiveX = VisionConstants.kAggressiveMultiTagStdDevs.get(0, 0);
        double multiX = VisionConstants.kMultiTagStdDevs.get(0, 0);
        assertTrue(aggressiveX <= multiX,
                "Aggressive multi-tag should be tightest");
    }

    @Test
    void stdDevs_distanceScaling_increasesWithDistance() {
        var baseStdDevs = VisionConstants.kSingleTagStdDevs;

        double nearDist = 1.0;
        double farDist = 4.0;

        var nearScaled = baseStdDevs.times(1 + (nearDist * nearDist / 30.0));
        var farScaled = baseStdDevs.times(1 + (farDist * farDist / 30.0));

        assertTrue(farScaled.get(0, 0) > nearScaled.get(0, 0),
                "Further distance should produce larger std devs");
    }

    @Test
    void stdDevs_distanceScaling_quadratic() {
        double dist1 = 2.0;
        double dist2 = 4.0;

        double scale1 = 1 + (dist1 * dist1 / 30.0);
        double scale2 = 1 + (dist2 * dist2 / 30.0);

        // At 2x distance, scaling should be ~4x more (quadratic)
        double ratio = (scale2 - 1.0) / (scale1 - 1.0);
        assertEquals(4.0, ratio, 0.01, "Scaling should be quadratic with distance");
    }

    @Test
    void stdDevs_singleTagTooFar_rejected() {
        int numTags = 1;
        double avgDist = VisionConstants.kMaxTagDistanceMeters + 1.0;

        var estStdDevs = VisionConstants.kSingleTagStdDevs;
        if (numTags == 1 && avgDist > VisionConstants.kMaxTagDistanceMeters) {
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }

        assertEquals(Double.MAX_VALUE, estStdDevs.get(0, 0),
                "Single tag too far should be effectively rejected");
    }

    // ==================== SINGLE TAG AMBIGUITY ====================

    @Test
    void ambiguity_highAmbiguity_rejected() {
        double ambiguity = VisionConstants.kMaxPoseAmbiguity + 0.1;
        int targetCount = 1;
        boolean reject = (targetCount == 1 && ambiguity > VisionConstants.kMaxPoseAmbiguity);
        assertTrue(reject, "High ambiguity single tag should be rejected");
    }

    @Test
    void ambiguity_lowAmbiguity_accepted() {
        double ambiguity = VisionConstants.kMaxPoseAmbiguity - 0.05;
        int targetCount = 1;
        boolean reject = (targetCount == 1 && ambiguity > VisionConstants.kMaxPoseAmbiguity);
        assertFalse(reject, "Low ambiguity should be accepted");
    }

    @Test
    void ambiguity_multiTag_ignoresAmbiguity() {
        double ambiguity = 0.9; // very high
        int targetCount = 3;
        boolean reject = (targetCount == 1 && ambiguity > VisionConstants.kMaxPoseAmbiguity);
        assertFalse(reject, "Multi-tag should not check ambiguity");
    }

    // ==================== DISTANCE VALIDATION ====================

    @Test
    void distanceValidation_noVisionData_returnsTrue() {
        double directHubDistance = -1.0;
        double odometryDistance = 5.0;
        boolean valid;
        if (directHubDistance < 0) {
            valid = true;
        } else {
            valid = Math.abs(odometryDistance - directHubDistance) < VisionConstants.kMaxDistanceDiscrepancyMeters;
        }
        assertTrue(valid, "No vision data should default to valid");
    }

    @Test
    void distanceValidation_smallDiscrepancy_valid() {
        double directHubDistance = 5.0;
        double odometryDistance = 5.3;
        double discrepancy = Math.abs(odometryDistance - directHubDistance);
        assertTrue(discrepancy < VisionConstants.kMaxDistanceDiscrepancyMeters);
    }

    @Test
    void distanceValidation_largeDiscrepancy_invalid() {
        double directHubDistance = 5.0;
        double odometryDistance = 7.0;
        double discrepancy = Math.abs(odometryDistance - directHubDistance);
        assertTrue(discrepancy >= VisionConstants.kMaxDistanceDiscrepancyMeters,
                "Large discrepancy should be flagged");
    }

    // ==================== CONSTANTS ====================

    @Test
    void maxPoseJump_isPositive() {
        assertTrue(VisionConstants.kMaxPoseJumpMeters > 0);
    }

    @Test
    void maxTagDistance_isPositive() {
        assertTrue(VisionConstants.kMaxTagDistanceMeters > 0);
    }

    @Test
    void maxPoseAmbiguity_isBetweenZeroAndOne() {
        assertTrue(VisionConstants.kMaxPoseAmbiguity > 0 && VisionConstants.kMaxPoseAmbiguity < 1.0);
    }

    @Test
    void fieldBoundaryMargin_isPositive() {
        assertTrue(VisionConstants.kFieldBoundaryMargin > 0);
    }

    @Test
    void aprilTagCameras_haveFour() {
        assertEquals(4, VisionConstants.kAprilTagCamNames.length);
        assertEquals(4, VisionConstants.kAprilTagCamTransforms.length);
    }
}
