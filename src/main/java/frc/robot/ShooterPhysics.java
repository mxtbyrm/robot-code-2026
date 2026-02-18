package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.constants.ShooterConstants;

/**
 * Basketball-arc shooting table generator for FRC 2026 REBUILT.
 *
 * <p>Models the shot like a basketball going into a basket — the ball must
 * arc UP, peak well above the Hub opening, and come DOWN into it from above.
 * A steeper descent angle means more of the opening is usable (like a swish),
 * while a flat entry reduces the effective window (like a brick off the rim).
 *
 * <h2>Basketball Arc Model:</h2>
 * <pre>
 *   x(t) = v · cos(θ) · t
 *   y(t) = h₀ + v · sin(θ) · t − ½ · g · t²
 *
 *   Apex height:    y_max = h₀ + (v·sinθ)² / (2g)
 *   Arrival time:   t_hub = d / (v·cosθ)
 *   Descent speed:  vy = v·sinθ − g·t_hub     (must be negative = descending)
 *   Descent angle:  α = atan(|vy| / vx)       (steeper = better)
 * </pre>
 *
 * <h2>Entry Window Geometry (top-down basketball view):</h2>
 * <pre>
 *   The ball enters the Hub opening from above at descent angle α.
 *   Effective vertical clearance = (D_hub − D_ball) · sin(α)
 *   Steeper α → more clearance → more forgiving shot.
 *   If α is too shallow, the ball skims the edge and bounces out.
 * </pre>
 *
 * <h2>Two Constraints (both must be satisfied):</h2>
 * <ol>
 *   <li><b>Trajectory constraint:</b> ball must arrive at Hub height at the
 *       correct horizontal distance → determines minimum velocity.</li>
 *   <li><b>Apex constraint:</b> ball must peak at least {@code kMinApexClearanceMeters}
 *       above the Hub → may require MORE velocity than the bare trajectory minimum,
 *       ensuring a clean descending arc (not a line drive).</li>
 * </ol>
 *
 * <h2>Tuning Guide:</h2>
 * <ol>
 *   <li>Measure {@code kShooterExitHeightMeters} — carpet to ball exit point</li>
 *   <li>Video the exit angles at min/max hood → set {@code kExitAngleAtMinHood/MaxHood}</li>
 *   <li>Adjust {@code kShooterEfficiencyFactor} until flywheel surface speed → ball speed ratio is correct</li>
 *   <li>Set {@code kBiasFactorNear} at min distance, {@code kBiasFactorFar} at max distance — mid-range interpolates automatically</li>
 *   <li>Increase {@code kMinApexClearanceMeters} if balls hit the rim instead of swishing in</li>
 *   <li>Check dashboard "ShooterPhysics/Descent Angle @Xm" — aim for >40° at all distances</li>
 * </ol>
 */
public final class ShooterPhysics {

    private ShooterPhysics() {} // utility class

    /** Standard gravitational acceleration (m/s²). */
    private static final double g = 9.81;

    /**
     * Compute the shooting table: distance → [flywheel RPS, hood angle].
     *
     * <p>For each distance step:
     * <ol>
     *   <li>Linearly interpolate hood angle across the distance range</li>
     *   <li>Map hood angle → ball exit angle</li>
     *   <li>Compute trajectory velocity (ball arrives at Hub height)</li>
     *   <li>Compute apex velocity (ball peaks above Hub by clearance margin)</li>
     *   <li>Take the HIGHER of the two → ensures both arc and accuracy</li>
     *   <li>Convert velocity → flywheel RPS via efficiency factor</li>
     * </ol>
     *
     * @return double[n][3] where each row = { distance_m, flywheel_rps, hood_angle_deg }
     */
    public static double[][] computeShootingTable() {
        double minDist = ShooterConstants.kMinShootingDistanceMeters;
        double maxDist = ShooterConstants.kMaxShootingDistanceMeters;
        double step = ShooterConstants.kTableDistanceStepMeters;
        double hubH = ShooterConstants.kHubHeightMeters;
        double exitH = ShooterConstants.kShooterExitHeightMeters;
        double deltaH = hubH - exitH;

        int numSteps = (int) Math.floor((maxDist - minDist) / step);
        // Add one extra entry if the last step doesn't land exactly on maxDist
        boolean addMax = (minDist + numSteps * step) < maxDist - 0.001;
        int numEntries = numSteps + 1 + (addMax ? 1 : 0);
        double[][] table = new double[numEntries][3];

        for (int i = 0; i < numEntries; i++) {
            double distance = Math.min(minDist + i * step, maxDist);

            // Hood angle: linearly interpolated from min to max across distance range
            double t = (distance - minDist) / (maxDist - minDist);
            double hoodAngle = ShooterConstants.kHoodMinAngleDegrees
                    + t * (ShooterConstants.kHoodMaxAngleDegrees - ShooterConstants.kHoodMinAngleDegrees);

            double exitAngleDeg = hoodToExitAngle(hoodAngle);
            double exitAngleRad = Math.toRadians(exitAngleDeg);

            // Constraint 1: trajectory — ball must arrive at hub height
            double vTrajectory = computeTrajectoryVelocity(distance, exitAngleRad, deltaH);

            // Constraint 2: apex — ball must peak above hub by clearance margin
            double vApex = computeApexVelocity(exitAngleRad, deltaH + ShooterConstants.kMinApexClearanceMeters);

            // Take the higher velocity — satisfies BOTH constraints
            double exitVelocity = Math.max(vTrajectory, vApex);

            // Apply distance-dependent real-world bias (more drag at longer range)
            exitVelocity *= biasFactorAtDistance(distance, minDist, maxDist);

            // Convert exit velocity → flywheel RPS
            double flywheelRPS = velocityToRPS(exitVelocity);

            table[i][0] = roundTo1(distance);
            table[i][1] = roundTo1(flywheelRPS);
            table[i][2] = roundTo1(hoodAngle);
        }

        // Validate: warn if any entry hit the fallback velocity path
        for (double[] entry : table) {
            double roundTrippedVelocity = rpsToVelocity(entry[1]);
            if (roundTrippedVelocity < 5.0 || entry[1] < 10.0) {
                DriverStation.reportWarning(
                        "[ShooterPhysics] Suspicious table entry at d=" + entry[0]
                        + "m: RPS=" + entry[1] + " — possible fallback hit", false);
            }
        }

        return table;
    }

    /**
     * Compute time-of-flight table: distance → seconds.
     *
     * <p>ToF is the time from ball exit to arriving at Hub height on the
     * descending arc. Uses the same dual-constraint velocity as the shooting table.
     *
     * @return double[n][2] where each row = { distance_m, tof_seconds }
     */
    public static double[][] computeTimeOfFlightTable() {
        double minDist = ShooterConstants.kMinShootingDistanceMeters;
        double maxDist = ShooterConstants.kMaxShootingDistanceMeters;
        double step = ShooterConstants.kTableDistanceStepMeters;
        double deltaH = ShooterConstants.kHubHeightMeters - ShooterConstants.kShooterExitHeightMeters;

        int numSteps = (int) Math.floor((maxDist - minDist) / step);
        boolean addMax = (minDist + numSteps * step) < maxDist - 0.001;
        int numEntries = numSteps + 1 + (addMax ? 1 : 0);
        double[][] table = new double[numEntries][2];

        for (int i = 0; i < numEntries; i++) {
            double distance = Math.min(minDist + i * step, maxDist);

            double t = (distance - minDist) / (maxDist - minDist);
            double hoodAngle = ShooterConstants.kHoodMinAngleDegrees
                    + t * (ShooterConstants.kHoodMaxAngleDegrees - ShooterConstants.kHoodMinAngleDegrees);

            double exitAngleRad = Math.toRadians(hoodToExitAngle(hoodAngle));
            double vTraj = computeTrajectoryVelocity(distance, exitAngleRad, deltaH);
            double vApex = computeApexVelocity(exitAngleRad, deltaH + ShooterConstants.kMinApexClearanceMeters);
            double exitVelocity = Math.max(vTraj, vApex);

            // Apply distance-dependent real-world bias (same as shooting table)
            exitVelocity *= biasFactorAtDistance(distance, minDist, maxDist);

            // Round-trip through RPS conversion to match the actual ball speed
            // the robot will produce (shooting table rounds RPS to 1 decimal)
            double roundedRPS = roundTo1(velocityToRPS(exitVelocity));
            double actualExitVelocity = rpsToVelocity(roundedRPS);

            // Time to reach hub horizontal distance using actual ball speed
            double tof = distance / (actualExitVelocity * Math.cos(exitAngleRad));

            table[i][0] = roundTo1(distance);
            table[i][1] = roundTo2(tof);
        }

        return table;
    }

    /**
     * Compute the descent angle at the Hub for a given shot.
     *
     * <p>This is the angle (degrees from horizontal) at which the ball is
     * descending when it crosses the Hub plane. Steeper = better:
     * <ul>
     *   <li>>50° = swish (clean entry, very forgiving)</li>
     *   <li>40-50° = good (reliable with the wide Hub opening)</li>
     *   <li>30-40° = marginal (may clip the edge)</li>
     *   <li>&lt;30° = bad (line drive, high bounce-out risk)</li>
     * </ul>
     *
     * @param distance horizontal distance to hub (m)
     * @param exitVelocity ball exit speed (m/s)
     * @param exitAngleRad ball exit angle from horizontal (radians)
     * @return descent angle in degrees (positive = descending)
     */
    public static double computeDescentAngle(double distance, double exitVelocity, double exitAngleRad) {
        double vx = exitVelocity * Math.cos(exitAngleRad);
        double vy0 = exitVelocity * Math.sin(exitAngleRad);
        double tHub = distance / vx;
        double vyAtHub = vy0 - g * tHub; // negative if descending

        if (vyAtHub >= 0) return 0; // still ascending — shouldn't happen with proper config
        return Math.toDegrees(Math.atan(Math.abs(vyAtHub) / vx));
    }

    /**
     * Compute the apex (peak) height of the trajectory above the floor.
     *
     * @param exitVelocity ball exit speed (m/s)
     * @param exitAngleRad ball exit angle from horizontal (radians)
     * @return apex height above floor (m)
     */
    public static double computeApexHeight(double exitVelocity, double exitAngleRad) {
        double vy0 = exitVelocity * Math.sin(exitAngleRad);
        return ShooterConstants.kShooterExitHeightMeters + (vy0 * vy0) / (2.0 * g);
    }

    /**
     * Compute the effective entry window at the Hub for a given descent angle.
     *
     * <p>The ball enters the circular Hub opening from above at angle α.
     * The "effective window" is how much lateral error is tolerable:
     * <pre>
     *   window = (D_hub − D_ball) / 2
     * </pre>
     * At steep angles this full window is available. At shallow angles only
     * a fraction is usable because the ball's trajectory clips the rim.
     * The vertical clearance scales with sin(α).
     *
     * @param descentAngleDeg descent angle in degrees
     * @return effective entry margin on each side (m)
     */
    public static double computeEntryMargin(double descentAngleDeg) {
        double hubDiameter = ShooterConstants.kHubInnerDiameterMeters;
        double ballDiameter = ShooterConstants.kBallDiameterMeters;
        double geometricMargin = (hubDiameter - ballDiameter) / 2.0;
        // At 90° descent (straight down), full margin is available.
        // At shallower angles, the ball's trajectory sweeps through more of the opening.
        return geometricMargin * Math.sin(Math.toRadians(descentAngleDeg));
    }

    // ==================== INTERNAL HELPERS ====================

    /**
     * Compute the distance-dependent real-world bias factor.
     *
     * <p>Linearly interpolates between {@code kBiasFactorNear} (at min distance)
     * and {@code kBiasFactorFar} (at max distance). Air drag, spin effects, and
     * foam losses compound over longer flight times, so far shots typically need
     * a higher correction than close shots.
     *
     * @param distance current shot distance (m)
     * @param minDist  minimum shooting distance (m) — corresponds to kBiasFactorNear
     * @param maxDist  maximum shooting distance (m) — corresponds to kBiasFactorFar
     * @return bias multiplier (clamped to [near, far] range)
     */
    static double biasFactorAtDistance(double distance, double minDist, double maxDist) {
        double range = maxDist - minDist;
        if (range <= 0) return ShooterConstants.kBiasFactorNear;
        double t = Math.max(0, Math.min(1, (distance - minDist) / range));
        return ShooterConstants.kBiasFactorNear
                + t * (ShooterConstants.kBiasFactorFar - ShooterConstants.kBiasFactorNear);
    }

    /**
     * Map hood mechanical angle to ball exit angle (degrees from horizontal).
     * <ul>
     *   <li>Hood at min (open) → max exit angle (steep, close range)</li>
     *   <li>Hood at max (closed) → min exit angle (flatter, far range)</li>
     * </ul>
     */
    static double hoodToExitAngle(double hoodAngleDeg) {
        double hoodRange = ShooterConstants.kHoodMaxAngleDegrees - ShooterConstants.kHoodMinAngleDegrees;
        double exitRange = ShooterConstants.kExitAngleAtMinHood - ShooterConstants.kExitAngleAtMaxHood;
        double hoodFraction = (hoodAngleDeg - ShooterConstants.kHoodMinAngleDegrees) / hoodRange;
        return ShooterConstants.kExitAngleAtMinHood - hoodFraction * exitRange;
    }

    /**
     * Compute the exit velocity so the ball arrives at hub height at the given distance.
     *
     * <p>Projectile equation solved for v:
     * <pre>
     *   v = d / (cosθ · √(2·(d·tanθ − Δh) / g))
     * </pre>
     *
     * <p>The term {@code d·tanθ − Δh} represents how far above the target the
     * unimpeded trajectory would go. It must be positive — if the exit angle is
     * too flat, the ball can't reach the Hub height even at infinite speed.
     */
    static double computeTrajectoryVelocity(double distance, double angleRad, double deltaH) {
        double tanTheta = Math.tan(angleRad);
        double cosTheta = Math.cos(angleRad);
        double excess = distance * tanTheta - deltaH;

        if (excess <= 0) {
            DriverStation.reportWarning(
                    "[ShooterPhysics] Exit angle too flat for d=" + distance
                    + "m, θ=" + String.format("%.1f", Math.toDegrees(angleRad))
                    + "°. Returning fallback velocity.", false);
            return 20.0;
        }

        return (distance / cosTheta) * Math.sqrt(g / (2.0 * excess));
    }

    /**
     * Compute the minimum exit velocity so the trajectory peaks at a given height
     * above the shooter exit point.
     *
     * <p>From the apex equation:
     * <pre>
     *   y_apex = h₀ + (v·sinθ)² / (2g)
     *   Required: y_apex ≥ h₀ + requiredClimb
     *   ∴ v ≥ √(2g · requiredClimb) / sinθ
     * </pre>
     *
     * @param angleRad      ball exit angle from horizontal (radians)
     * @param requiredClimb minimum height gain above exit point (m).
     *                      For basketball-style shots this is Δh + apex clearance.
     * @return minimum exit velocity (m/s) to reach the required apex
     */
    static double computeApexVelocity(double angleRad, double requiredClimb) {
        if (requiredClimb <= 0) return 0;
        double sinTheta = Math.sin(angleRad);
        if (sinTheta <= 0) return 20.0; // fallback for invalid angle
        return Math.sqrt(2.0 * g * requiredClimb) / sinTheta;
    }

    /**
     * Convert ball exit velocity (m/s) to bottom flywheel RPS.
     *
     * <p>The shooter has dual flywheels (bottom 4″ + top 2″) geared so their
     * surface speeds are equal. The ball is squeezed between both wheels:
     * <pre>
     *   surface_speed = RPS_bottom × π × D_bottom
     *                 = RPS_top    × π × D_top     (equal by gearing)
     *   exit_speed    = surface_speed × efficiency
     *   ∴ RPS_bottom  = exit_speed / (π × D_bottom × efficiency)
     * </pre>
     * We return bottom flywheel RPS because that's what the motor encoder reads.
     */
    private static double velocityToRPS(double exitVelocity) {
        return exitVelocity
                / (Math.PI * ShooterConstants.kBottomFlywheelDiameterMeters
                   * ShooterConstants.kShooterEfficiencyFactor);
    }

    /**
     * Convert flywheel RPS back to ball exit velocity (m/s).
     * Inverse of {@link #velocityToRPS}. Used to compute the actual ball speed
     * after RPS rounding, so ToF and other calculations match real behavior.
     */
    static double rpsToVelocity(double rps) {
        return rps * Math.PI * ShooterConstants.kBottomFlywheelDiameterMeters
               * ShooterConstants.kShooterEfficiencyFactor;
    }

    /** Round to 1 decimal place. */
    private static double roundTo1(double value) {
        return Math.round(value * 10.0) / 10.0;
    }

    /** Round to 2 decimal places. */
    private static double roundTo2(double value) {
        return Math.round(value * 100.0) / 100.0;
    }
}
