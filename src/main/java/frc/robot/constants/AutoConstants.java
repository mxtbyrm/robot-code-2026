package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// ==================== AUTONOMOUS CONSTANTS ====================
public final class AutoConstants {

    // ==================== FIELD DIMENSIONS (Game Manual Section 5.2) ====================
    /** Field length: 651.2 inches. */
    public static final double kFieldLengthMeters = Units.inchesToMeters(651.2);
    /** Field width: 317.7 inches. */
    public static final double kFieldWidthMeters = Units.inchesToMeters(317.7);

    // ==================== HUB DISTANCE FROM ALLIANCE WALL ====================
    // Moved here from HubConstants to break the circular dependency between
    // HubConstants (needs field dims) and AutoConstants.PathfindingConstants (needs hub distance).
    // Field: 16.54m × 8.07m
    // Each HUB is centered along field width, 158.6in (~4.03m) from its alliance wall.
    public static final double kDistanceFromAllianceWallMeters = Units.inchesToMeters(158.6); // ~4.028m

    // ==================== AUTO PERIOD (Game Manual Section 6.4) ====================
    /** Autonomous period: first 20 seconds of the match. Both HUBs active. */
    public static final double kAutoDurationSeconds = 20.0;

    // ==================== MATCH TIMING SUMMARY (Game Manual Table 6-2) ====================
    // AUTO:            20s   (both HUBs active)
    // TRANSITION SHIFT: 10s  (both HUBs active)
    // SHIFT 1-4:       4×25s (alternating active/inactive)
    // END GAME:         30s  (both HUBs active)
    // Total match:      160s (2:40)

    // ==================== SHOOTING TIMING ====================
    /** Max time to wait for shooter lock-on (flywheel + turret + hood at setpoint). */
    public static final double kShooterLockTimeoutSeconds = 1.0;

    /** Time to feed all 8 preloaded FUEL through the shooter (seconds). */
    public static final double kPreloadShootSeconds = 4.0;

    /** Time to feed a collected batch of FUEL through the shooter (seconds). */
    public static final double kCollectedShootSeconds = 3.0;

    /** Max time for vision-assisted ball chase before giving up and continuing auto. */
    public static final double kVisionChaseTimeoutSeconds = 2.0;

    // ==================== AUTO ROBUSTNESS TIMEOUTS ====================
    /** Maximum time for any single ChoreoLib trajectory before aborting. */
    public static final double kMaxTrajectoryTimeoutSeconds = 6.0;
    /** Maximum time for a shoot sequence (wait-for-lock + feed) before continuing. */
    public static final double kMaxShootTimeoutSeconds = 3.0;
    /** Maximum time for vision-assisted ball collection in auto. */
    public static final double kMaxVisionCollectTimeoutSeconds = 4.0;

    // ==================== PATHPLANNER ON-THE-FLY PATHFINDING ====================
    public static final class PathfindingConstants {

        // ---- Path constraints ----
        /** Maximum velocity during pathfinding (m/s). Conservative for auto accuracy. */
        public static final double kMaxVelocity = 3.0;
        /** Maximum acceleration during pathfinding (m/s²). */
        public static final double kMaxAcceleration = 2.5;
        /** Maximum angular velocity during pathfinding (rad/s). */
        public static final double kMaxAngularVelocity = Math.toRadians(540.0);
        /** Maximum angular acceleration during pathfinding (rad/s²). */
        public static final double kMaxAngularAcceleration = Math.toRadians(720.0);

        // ---- Pathfinding timeouts ----
        /** Max time for a pathfind command before it's considered stuck (seconds). */
        public static final double kPathfindTimeoutSeconds = 6.0;
        /** Shorter timeout for nearby movements like shooting range (seconds). */
        public static final double kShortPathfindTimeoutSeconds = 4.0;

        // ---- Trench geometry (Game Manual §5.6) ----
        // Trenches extend from guardrail to bump on both sides of the field.
        // Blue alliance origin: (0,0) is at the blue alliance wall, near-guardrail corner.
        // Field: 16.54m × 8.07m

        // The trench structure sits on both guardrail sides.
        // Near-guardrail trenches (Y close to 0 and Y close to 8.07m)
        /** Trench overall width: 65.65in = 1.668m */
        public static final double kTrenchWidthMeters = Units.inchesToMeters(65.65);
        /** Trench depth (field-lengthwise): 47.0in = 1.194m */
        public static final double kTrenchDepthMeters = Units.inchesToMeters(47.0);
        /** Passable clearance under trench arm: 22.25in = 0.5652m */
        public static final double kTrenchClearanceMeters = Units.inchesToMeters(22.25);
        /** Passable width under trench arm: 50.34in = 1.279m */
        public static final double kTrenchPassableWidthMeters = Units.inchesToMeters(50.34);

        // Trench Y-axis center positions (perpendicular to alliance wall)
        // Near-side trenches sit against the guardrails (Y ≈ 0.6m and Y ≈ 7.47m)
        /** Y-center of the near-guardrail trench (blue-left side). */
        public static final double kTrenchNearGuardrailY = kTrenchWidthMeters / 2.0; // ~0.834m
        /** Y-center of the far-guardrail trench (blue-right side). */
        public static final double kTrenchFarGuardrailY = kFieldWidthMeters - kTrenchWidthMeters / 2.0; // ~7.236m

        // Trench X-axis span: from the bump to the guardrail, across the alliance/neutral boundary.
        // Each alliance has 2 trenches (one per guardrail side). They span from the bump
        // (which is at ~4.03m from alliance wall) extending the trench depth further into the neutral zone.
        // Blue alliance side: X ≈ kDistanceFromAllianceWall - bumpDepth/2 through the bump region
        // Approximate X-span for blue-side trench underpass center
        /** X-center of the blue alliance trench underpass. */
        public static final double kBlueTrenchCenterX = kDistanceFromAllianceWallMeters; // ~4.03m
        /** X-center of the red alliance trench underpass. */
        public static final double kRedTrenchCenterX = kFieldLengthMeters - kDistanceFromAllianceWallMeters; // ~12.51m

        // ---- Depot positions (Game Manual §5.7) ----
        // Depots are along the alliance wall, 42in wide × 27in deep.
        // Blue depot is on the near-guardrail side of the blue alliance wall.
        /** Blue depot center position (near alliance wall, near-guardrail side). */
        public static final Translation2d kBlueDepotCenter = new Translation2d(
                Units.inchesToMeters(27.0 / 2.0), // ~0.34m from wall
                Units.inchesToMeters(42.0 / 2.0)); // ~0.53m from guardrail
        /** Red depot center position (mirrored). */
        public static final Translation2d kRedDepotCenter = new Translation2d(
                kFieldLengthMeters - Units.inchesToMeters(27.0 / 2.0),
                kFieldWidthMeters - Units.inchesToMeters(42.0 / 2.0));

        // ---- Key autonomous poses (blue alliance, mirrored for red) ----
        /** Optimal shooting distance from hub center (meters). */
        public static final double kOptimalShootingDistanceMeters = 2.5;
        /** Neutral zone center for collection. */
        public static final Translation2d kNeutralZoneCenter = new Translation2d(
                kFieldLengthMeters / 2.0, kFieldWidthMeters / 2.0);
        /** Neutral zone near-trench collection point (near-guardrail side). */
        public static final Translation2d kNeutralZoneNearTrench = new Translation2d(
                kFieldLengthMeters / 2.0, kTrenchNearGuardrailY);
        /** Neutral zone far-trench collection point (far-guardrail side). */
        public static final Translation2d kNeutralZoneFarTrench = new Translation2d(
                kFieldLengthMeters / 2.0, kTrenchFarGuardrailY);
    }
}
