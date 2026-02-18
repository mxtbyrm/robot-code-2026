package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

// ==================== HUB — FIELD STRUCTURE CONSTANTS ====================
// The HUB is the primary scoring target in 2026 REBUILT™.
// It is a multi-stage rectangular prism located in the NEUTRAL ZONE.
// FUEL (foam balls) are scored through a hexagonal opening at the top.
// Scored FUEL is distributed back onto the field through 4 base exits.
public final class HubConstants {

    // ==================== PHYSICAL DIMENSIONS ====================
    /** Base footprint: 47in × 47in square on carpet */
    public static final double kBaseWidthMeters = Units.inchesToMeters(47.0);  // ~1.194m
    public static final double kBaseLengthMeters = Units.inchesToMeters(47.0); // ~1.194m

    /** Front edge of scoring opening: 72in above the floor */
    public static final double kScoringOpeningHeightMeters = Units.inchesToMeters(72.0); // ~1.829m

    /** Hexagonal scoring opening width: 41.7in */
    public static final double kScoringOpeningWidthMeters = Units.inchesToMeters(41.7); // ~1.059m

    /** Number of FUEL exits at the base of the HUB */
    public static final int kBaseExitCount = 4;

    // ==================== FIELD POSITIONS (alliance-aware) ====================
    // Field: 16.54m × 8.07m
    // Each HUB is centered along field width, 158.6in (~4.03m) from its alliance wall.
    // kDistanceFromAllianceWallMeters has moved to AutoConstants to avoid a circular dependency.
    public static final Translation2d kBlueHubCenter = new Translation2d(
            AutoConstants.kDistanceFromAllianceWallMeters,
            AutoConstants.kFieldWidthMeters / 2.0);
    public static final Translation2d kRedHubCenter = new Translation2d(
            AutoConstants.kFieldLengthMeters - AutoConstants.kDistanceFromAllianceWallMeters,
            AutoConstants.kFieldWidthMeters / 2.0);

    /** Returns the active alliance's HUB center position at runtime. */
    public static Translation2d getActiveHubPosition() {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            return kRedHubCenter;
        }
        return kBlueHubCenter;
    }

    // ==================== APRILTAG MARKERS ====================
    // The HUB has 16 unique AprilTags (2 per face × 4 faces per HUB × 2 HUBs)
    // Tag centers are 44.25in (1.124m) off the floor.
    public static final double kAprilTagHeightMeters = Units.inchesToMeters(44.25); // 1.124m

    /** Blue alliance HUB AprilTag IDs (2 tags per face × 4 faces) */
    public static final int[] kBlueHubTagIds = { 2, 3, 4, 5, 8, 9, 10, 11 };
    /** Red alliance HUB AprilTag IDs (2 tags per face × 4 faces) */
    public static final int[] kRedHubTagIds  = { 18, 19, 20, 21, 24, 25, 26, 27 };
    /** All HUB AprilTag IDs across both alliances */
    public static final int[] kAllHubTagIds  = {
        2, 3, 4, 5, 8, 9, 10, 11,
        18, 19, 20, 21, 24, 25, 26, 27
    };

    /** Returns the HUB AprilTag IDs for the current alliance. */
    public static int[] getActiveHubTagIds() {
        var alliance = edu.wpi.first.wpilibj.DriverStation.getAlliance();
        if (alliance.isPresent() && alliance.get() == edu.wpi.first.wpilibj.DriverStation.Alliance.Red) {
            return kRedHubTagIds;
        }
        return kBlueHubTagIds;
    }

    /** Check if a given AprilTag ID belongs to any HUB. */
    public static boolean isHubTag(int tagId) {
        for (int id : kAllHubTagIds) {
            if (id == tagId) return true;
        }
        return false;
    }

    /** Check if a given AprilTag ID belongs to the current alliance's HUB. */
    public static boolean isOwnHubTag(int tagId) {
        for (int id : getActiveHubTagIds()) {
            if (id == tagId) return true;
        }
        return false;
    }

    // ==================== SCORING RULES (Game Manual Section 6.5, Table 6-4) ====================
    // Active HUB:   ALLIANCE-colored DMX light bars ON  → scored FUEL = 1 point
    // Inactive HUB: DMX light bars OFF                  → scored FUEL = 0 points
    //
    // During ALLIANCE SHIFTS in Teleop, active/inactive alternates every 25 seconds.
    // TRANSITION SHIFT (first 10s): both HUBs active.
    // END GAME (last 30s): both HUBs active.
    // The alliance that scored MORE FUEL in Autonomous starts with HUB
    // INACTIVE for Shift 1.
    //
    // FUEL scored continues to count for up to 3 seconds after HUB deactivates
    // (processing time grace period).
    /** Match points per FUEL scored while HUB is Active */
    public static final int kActiveHubPointsPerFuel = 1;
    /** Match points per FUEL scored while HUB is Inactive */
    public static final int kInactiveHubPointsPerFuel = 0;

    // ==================== TOWER SCORING (Game Manual Table 6-4) ====================
    /** LEVEL 1 climb: 15 pts in AUTO (max 2 robots), 10 pts in TELEOP */
    public static final int kLevel1AutoPoints = 15;
    public static final int kLevel1TeleopPoints = 10;
    /** LEVEL 2 climb: 20 pts (TELEOP only) */
    public static final int kLevel2TeleopPoints = 20;
    /** LEVEL 3 climb: 30 pts (TELEOP only) */
    public static final int kLevel3TeleopPoints = 30;

    // ==================== RANKING POINT THRESHOLDS (Game Manual Table 6-5) ====================
    // Regional / District Events thresholds (may increase at Championships)
    /** ENERGIZED RP: ≥ 100 FUEL scored in active HUBs during the match */
    public static final int kEnergizedRpThreshold = 100;
    /** SUPERCHARGED RP: ≥ 360 FUEL scored in active HUBs during the match */
    public static final int kSuperchargedRpThreshold = 360;
    /** TRAVERSAL RP: ≥ 50 total TOWER points during the match */
    public static final int kTraversalRpThreshold = 50;

    // ==================== MATCH FUEL STAGING (Game Manual Section 6.3.4) ====================
    /** Maximum FUEL preloaded per robot */
    public static final int kMaxPreloadPerRobot = 8;
    /** Total FUEL staged per match */
    public static final int kTotalFuelPerMatch = 504;
    /** FUEL per DEPOT */
    public static final int kFuelPerDepot = 24;
    /** FUEL per OUTPOST CHUTE */
    public static final int kFuelPerChute = 24;

    // ==================== ANTI-CAMPING (G408) ====================
    // ROBOTS may NOT catch FUEL directly from HUB exit chutes.
    // FUEL must touch the carpet or another object before gaining control.
    // Minimum standoff distance from HUB base exits for strategic positioning.
    public static final double kAntiCampStandoffMeters = 0.5;
}
