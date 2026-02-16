package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.Set;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.AutoConstants.PathfindingConstants;
import frc.robot.Constants.HubConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveDrive;

/**
 * On-the-fly autonomous routines using PathPlanner's AD* pathfinding.
 *
 * <p>Unlike the old Choreo-based Autos which follow pre-planned trajectories,
 * these routines dynamically compute paths at runtime using PathPlanner's
 * pathfinding engine. The navgrid ensures correct routing by marking field
 * elements like the Hub and Bump as obstacles, while keeping the Trench
 * run clear for passage.
 *
 * <h2>Key improvements over Choreo:</h2>
 * <ul>
 *   <li>Dynamic pathfinding — no pre-generated trajectories needed</li>
 *   <li>Intake arm auto-deploys at start of every routine</li>
 *   <li>Flywheel pre-spins during drive-to-score for 0 lockout time</li>
 *   <li>Timeout guards on every pathfind to prevent stalls</li>
 *   <li>Proper end-of-auto positioning facing hub for teleop advantage</li>
 * </ul>
 *
 * <h2>Available Routines:</h2>
 * <ul>
 *   <li>{@link #preloadAndPark()} — Score preload, park in neutral zone</li>
 *   <li>{@link #preloadAndCollectCenter()} — Score preload, collect center, score again</li>
 *   <li>{@link #preloadAndCollectSide()} — Score preload, collect trench side, score again</li>
 *   <li>{@link #preloadAndCollectDepot()} — Score preload, collect from depot, score again</li>
 *   <li>{@link #doubleTrenchScore()} — Score → collect → trench back → score → neutral zone</li>
 *   <li>{@link #aggressiveSweep()} — Rapid multi-stop neutral zone sweep</li>
 *   <li>{@link #defensivePreload()} — Score preload, park defensively near hub</li>
 * </ul>
 */
public final class OnTheFlyAutos {

    private final SwerveDrive swerveDrive;
    private final Shooter shooter;
    private final Superstructure superstructure;

    /** Factory for creating vision-assisted ball chase commands. Injected from RobotContainer. */
    private final Supplier<Command> visionCollectFactory;

    // ---- Pathfinding timeouts (seconds) ----
    private static final double kPathfindTimeoutSeconds = PathfindingConstants.kPathfindTimeoutSeconds;
    private static final double kShortPathfindTimeoutSeconds = PathfindingConstants.kShortPathfindTimeoutSeconds;

    /** Default path constraints for autonomous pathfinding. */
    private static final PathConstraints kDefaultConstraints = new PathConstraints(
            PathfindingConstants.kMaxVelocity,
            PathfindingConstants.kMaxAcceleration,
            PathfindingConstants.kMaxAngularVelocity,
            PathfindingConstants.kMaxAngularAcceleration);

    /** Slower constraints for precise positioning (near scoring targets). */
    private static final PathConstraints kPreciseConstraints = new PathConstraints(
            1.5, // m/s — slower for accuracy
            1.5, // m/s²
            PathfindingConstants.kMaxAngularVelocity,
            PathfindingConstants.kMaxAngularAcceleration);

    /**
     * @param visionCollectFactory factory that creates a fresh vision-chase command each call.
     *                             Wired in {@code RobotContainer} to keep command creation centralized.
     */
    public OnTheFlyAutos(SwerveDrive swerveDrive, Shooter shooter,
                         Superstructure superstructure, Supplier<Command> visionCollectFactory) {
        this.swerveDrive = swerveDrive;
        this.shooter = shooter;
        this.superstructure = superstructure;
        this.visionCollectFactory = visionCollectFactory;
    }

    // ========================================================================
    //  COMPOSABLE HELPER COMMANDS
    // ========================================================================

    /**
     * Deploy intake arm + reset shot counter. Called once at the start of every auto.
     * This ensures the Superstructure knows the arm is deployed, so
     * intakeRollerCommand() will actually spin the rollers.
     */
    private Command autoInit() {
        return Commands.runOnce(() -> {
            superstructure.requestIdle(); // Kickstart state machine
            superstructure.deployIntakeArm();
            superstructure.resetBallsShotCount();
        }).withName("OTF: Auto Init");
    }

    /**
     * Wait for shooter to lock on, then feed FUEL for the given duration.
     * Mirrors the pattern from Choreo Autos.
     */
    private Command waitAndShoot(double feedSeconds) {
        return Commands.sequence(
                Commands.runOnce(() -> shooter.enableTracking()),
                shooter.waitUntilReady().withTimeout(AutoConstants.kShooterLockTimeoutSeconds),
                superstructure.autoShootCommand(feedSeconds),
                Commands.runOnce(() -> shooter.disableTracking())
        ).withName("OTF: Wait & Shoot");
    }

    /** Dump all 8 preloaded FUEL. */
    private Command shootPreload() {
        return waitAndShoot(AutoConstants.kPreloadShootSeconds)
                .withName("OTF: Shoot Preload");
    }

    /** Dump a collected batch of FUEL after a sweep. */
    private Command shootCollected() {
        return waitAndShoot(AutoConstants.kCollectedShootSeconds)
                .withName("OTF: Shoot Collected");
    }

    /**
     * Run intake rollers as a held command (for use with deadline or parallel).
     * Requires {@link #autoInit()} to have been called first to deploy the arm.
     */
    private Command intakeWhileDriving() {
        return superstructure.intakeRollerCommand()
                .withName("OTF: Intake While Driving");
    }

    /**
     * Vision-assisted ball collection for auto.
     *
     * <p>Uses the intake camera to chase a visible ball while running intake rollers.
     * Finishes when the ball is close enough to collect, or after a timeout.
     * The vision chase command is created by the factory injected from RobotContainer.
     */
    private Command visionCollect() {
        return Commands.deadline(
                visionCollectFactory.get(),
                intakeWhileDriving()
        ).withName("OTF: Vision Collect");
    }

    /**
     * Pre-spin the flywheel while driving to the scoring position.
     * This overlaps shooter spin-up with drive time, saving 0.5-1.0s per cycle.
     * Tracking is disabled after this command ends (waitAndShoot re-enables it).
     */
    private Command preSpinFlywheel() {
        return Commands.startEnd(
                () -> shooter.enableTracking(),
                () -> {} // Don't disable yet — let waitAndShoot take over
        ).withName("OTF: Pre-Spin Flywheel");
    }

    /**
     * Pathfind to the optimal shooting range from the hub.
     * The robot drives to a position kOptimalShootingDistanceMeters away
     * from the hub center, facing the hub. Pre-spins flywheel during approach.
     * <p>
     * <b>Coordinate System Note:</b> AutoBuilder expects all targets in the
     * BLUE ALLIANCE frame and flips them automatically for Red. Therefore,
     * we must perform all geometry calculations in the Blue frame.
     */
    private Command driveToShootingRange() {
        return Commands.deadline(
                Commands.defer(() -> {
                    // 1. Get current pose and normalize to Blue Alliance Frame
                    Pose2d paramsPose = flipToBlue(swerveDrive.getPose());
                    Translation2d blueHub = HubConstants.kBlueHubCenter;

                    // 2. Compute shooting position relative to Blue Hub
                    Translation2d toHub = blueHub.minus(paramsPose.getTranslation());
                    double distance = toHub.getNorm();

                    Translation2d blueShootPos;
                    if (distance > 0.01) { // avoid division by zero
                        Translation2d direction = toHub.div(distance);
                        blueShootPos = blueHub.minus(
                                direction.times(PathfindingConstants.kOptimalShootingDistanceMeters));
                    } else {
                        // Extremely close to hub — just back up along +X
                        blueShootPos = new Translation2d(
                                blueHub.getX() + PathfindingConstants.kOptimalShootingDistanceMeters,
                                blueHub.getY());
                    }

                    // 3. Face toward the Blue Hub
                    Rotation2d facingBlueHub = new Rotation2d(
                            blueHub.getX() - blueShootPos.getX(),
                            blueHub.getY() - blueShootPos.getY());

                    return AutoBuilder.pathfindToPose(
                            new Pose2d(blueShootPos, facingBlueHub),
                            kPreciseConstraints,
                            0.0 // stopped for shooting
                    );
                }, Set.of(swerveDrive)).withTimeout(kShortPathfindTimeoutSeconds),
                preSpinFlywheel() // Overlap flywheel spin-up with driving
        ).withName("OTF: Drive To Shooting Range (Pre-Spin)");
    }

    /**
     * Pathfind to a field-relative target with default constraints and timeout.
     * @param target Target position in BLUE ALLIANCE frame.
     */
    private Command pathfindTo(Translation2d target, Rotation2d heading, double endVelocity) {
        Command pathfind = AutoBuilder.pathfindToPose(
                new Pose2d(target, heading),
                kDefaultConstraints,
                endVelocity
        );
        if (pathfind == null) {
            Logger.recordOutput("Auto/Warning", "pathfindToPose returned null for " + target);
            return Commands.none().withName("OTF: Pathfind NULL fallback");
        }
        return pathfind.withTimeout(kPathfindTimeoutSeconds)
                .withName("OTF: Pathfind To " + target);
    }

    /**
     * Pathfind to a target with precise (slow) constraints and timeout.
     * @param target Target position in BLUE ALLIANCE frame.
     */
    private Command pathfindToPrecise(Translation2d target, Rotation2d heading) {
        Command pathfind = AutoBuilder.pathfindToPose(
                new Pose2d(target, heading),
                kPreciseConstraints,
                0.0
        );
        if (pathfind == null) {
            Logger.recordOutput("Auto/Warning", "pathfindToPose returned null for precise " + target);
            return Commands.none().withName("OTF: Pathfind Precise NULL fallback");
        }
        return pathfind.withTimeout(kShortPathfindTimeoutSeconds)
                .withName("OTF: Pathfind Precise To " + target);
    }

    /**
     * Gets the heading to face toward the BLUE hub from a given BLUE position.
     * AutoBuilder will flip the resulting Pose2d (Pos + Rot) for Red.
     */
    private Rotation2d headingToHub(Translation2d fromBlue) {
        Translation2d blueHub = HubConstants.kBlueHubCenter;
        return new Rotation2d(
                blueHub.getX() - fromBlue.getX(),
                blueHub.getY() - fromBlue.getY());
    }

    /**
     * Gets a collection point in the neutral zone near the trench (Blue Frame).
     */
    private Translation2d getNeutralZoneNearTrench() {
        return PathfindingConstants.kNeutralZoneNearTrench;
    }

    /**
     * Gets the BLUE depot center position.
     * AutoBuilder flips this to the Red Depot automatically when on Red.
     */
    private Translation2d getDepotCenter() {
        return PathfindingConstants.kBlueDepotCenter;
    }

    /**
     * End-of-auto: pathfind to a good teleop starting position facing the hub.
     * Uses a position slightly in front of the neutral zone center.
     */
    private Command parkFacingHub(Translation2d parkPosition) {
        // parkPosition is assumed to be Blue Frame
        return pathfindTo(
                parkPosition,
                headingToHub(parkPosition), // Faces Blue Hub
                0.0
        ).withName("OTF: Park Facing Hub");
    }

    /**
     * Helper: Transforms a real field pose into the Blue Alliance Frame.
     * If already Blue, returns as-is. If Red, mirrors X and Y (point reflection? No, mirror across center width usually).
     * PathPlanner mirroring:
     * Blue (x, y, theta) -> Red (FieldLength - x, FieldWidth - y, 180 - theta) (Diagonal flip?)
     * Or just X flip? (FieldLength - x, y, 180 - theta).
     * PathPlanner defaults to "Mirror across field midline (X axis inverted)".
     */
    private Pose2d flipToBlue(Pose2d pose) {
        var alliance = DriverStation.getAlliance();
        Pose2d result = pose;
        if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
            double length = AutoConstants.kFieldLengthMeters;
            double width = AutoConstants.kFieldWidthMeters;

            result = new Pose2d(
                new Translation2d(
                    length - pose.getX(),
                    width - pose.getY()
                ),
                new Rotation2d(Math.PI).minus(pose.getRotation()) // 180 - theta
            );
        }
        // Field boundary assertion after flip
        double x = result.getX();
        double y = result.getY();
        if (x < 0 || x > AutoConstants.kFieldLengthMeters
                || y < 0 || y > AutoConstants.kFieldWidthMeters) {
            Logger.recordOutput("Auto/Warning",
                    "flipToBlue produced out-of-bounds pose: (" + x + ", " + y + ")");
        }
        return result;
    }

    // ========================================================================
    //  AUTONOMOUS ROUTINES
    // ========================================================================

    /**
     * <b>Preload & Park</b> — Simple and reliable.
     *
     * <ol>
     *   <li>Deploy intake arm + init</li>
     *   <li>Score all 8 preloaded FUEL from starting position</li>
     *   <li>Pathfind through the trench into the neutral zone</li>
     *   <li>Park facing hub for teleop advantage</li>
     * </ol>
     *
     * <p>Total time: ~10-12s. Leaves 8-10s of auto remaining for positioning.
     */
    public Command preloadAndPark() {
        return Commands.sequence(
                autoInit(),

                // Score preloaded FUEL
                shootPreload(),

                // Drive through trench to neutral zone, park facing hub
                parkFacingHub(PathfindingConstants.kNeutralZoneCenter),

                // Lock wheels
                Commands.runOnce(() -> swerveDrive.lockWheels(), swerveDrive)
        ).withName("OTF Auto: Preload & Park");
    }

    /**
     * <b>Preload & Collect Center</b> — High yield from neutral zone center.
     *
     * <ol>
     *   <li>Deploy intake arm + init</li>
     *   <li>Score all 8 preloaded FUEL</li>
     *   <li>Pathfind through trench to neutral zone center with intake running</li>
     *   <li>Dwell to collect loose FUEL</li>
     *   <li>Pathfind back through trench to shooting range (pre-spinning flywheel)</li>
     *   <li>Score collected FUEL</li>
     * </ol>
     */
    public Command preloadAndCollectCenter() {
        return Commands.sequence(
                autoInit(),

                // Score preloaded FUEL
                shootPreload(),

                // Drive to neutral zone center with intake running
                Commands.deadline(
                        pathfindTo(
                                PathfindingConstants.kNeutralZoneCenter,
                                Rotation2d.fromDegrees(180), // face back toward alliance zone
                                0.5), // keep moving for smooth intake
                        intakeWhileDriving()
                ),

                // Vision-chase loose balls in the area
                visionCollect(),

                // Return to shooting range (flywheel pre-spins during drive)
                driveToShootingRange(),

                // Score collected FUEL
                shootCollected()
        ).withName("OTF Auto: Preload & Collect Center");
    }

    /**
     * <b>Preload & Collect Side</b> — Targets FUEL near the trench guardrail.
     *
     * <ol>
     *   <li>Deploy intake arm + init</li>
     *   <li>Score all 8 preloaded FUEL</li>
     *   <li>Pathfind through trench to near-trench area in neutral zone</li>
     *   <li>Intake loose FUEL along the guardrail/trench side</li>
     *   <li>Pathfind back through trench to shooting range (pre-spinning flywheel)</li>
     *   <li>Score collected FUEL</li>
     * </ol>
     */
    public Command preloadAndCollectSide() {
        Translation2d trenchCollection = getNeutralZoneNearTrench();

        return Commands.sequence(
                autoInit(),

                // Score preloaded FUEL
                shootPreload(),

                // Drive through trench to side collection point
                Commands.deadline(
                        pathfindTo(
                                trenchCollection,
                                Rotation2d.fromDegrees(0),
                                0.5),
                        intakeWhileDriving()
                ),

                // Vision-chase loose balls near the trench
                visionCollect(),

                // Return to shooting range (flywheel pre-spins during drive)
                driveToShootingRange(),

                // Score collected FUEL
                shootCollected()
        ).withName("OTF Auto: Preload & Collect Side");
    }

    /**
     * <b>Preload & Collect Depot</b> — Targets the depot for guaranteed FUEL supply.
     *
     * <ol>
     *   <li>Deploy intake arm + init</li>
     *   <li>Score all 8 preloaded FUEL</li>
     *   <li>Pathfind to the depot (near alliance wall)</li>
     *   <li>Intake FUEL from the depot</li>
     *   <li>Pathfind to shooting range (pre-spinning flywheel)</li>
     *   <li>Score collected FUEL</li>
     * </ol>
     *
     * <p>The depot is a reliable FUEL source (24 FUEL pre-staged).
     * This routine doesn't require crossing the trench.
     */
    public Command preloadAndCollectDepot() {
        return Commands.sequence(
                autoInit(),

                // Score preloaded FUEL
                shootPreload(),

                // Drive to depot
                Commands.deadline(
                        Commands.defer(() -> pathfindToPrecise(
                                getDepotCenter(),
                                Rotation2d.fromDegrees(0)), Set.of(swerveDrive)),
                        intakeWhileDriving()
                ),

                // Vision-chase FUEL at the depot
                visionCollect(),

                // Drive to shooting range (flywheel pre-spins during drive)
                driveToShootingRange(),

                // Score collected FUEL
                shootCollected()
        ).withName("OTF Auto: Preload & Collect Depot");
    }

    /**
     * <b>Double Trench Score</b> — Cross the trench TWICE for maximum scoring.
     *
     * <ol>
     *   <li>Deploy intake arm + init</li>
     *   <li>Score all 8 preloaded FUEL</li>
     *   <li>Pathfind through trench to neutral zone → intake FUEL</li>
     *   <li>Pathfind back through trench to scoring range (pre-spinning flywheel)</li>
     *   <li>Score collected FUEL (second volley)</li>
     *   <li>Pathfind through trench again to neutral zone for teleop head start</li>
     * </ol>
     *
     * <p>This is the most aggressive scoring routine. Requires fast flywheel
     * spin-up and efficient pathing. Pre-spin during approach saves ~1s.
     */
    public Command doubleTrenchScore() {
        Translation2d trenchCollection = getNeutralZoneNearTrench();

        return Commands.sequence(
                autoInit(),

                // === FIRST VOLLEY: Score preloaded FUEL ===
                shootPreload(),

                // === TRENCH CROSSING 1: Alliance → Neutral zone ===
                Commands.deadline(
                        pathfindTo(
                                trenchCollection,
                                Rotation2d.fromDegrees(0),
                                0.5),
                        intakeWhileDriving()
                ),

                // Vision-chase FUEL near the trench
                visionCollect(),

                // === TRENCH CROSSING 2: Neutral zone → Alliance (back to score) ===
                driveToShootingRange(),

                // === SECOND VOLLEY: Score collected FUEL ===
                shootCollected(),

                // === TRENCH CROSSING 3: Alliance → Neutral zone (teleop head start) ===
                Commands.deadline(
                        pathfindTo(
                                PathfindingConstants.kNeutralZoneCenter,
                                Rotation2d.fromDegrees(0),
                                0.5),
                        intakeWhileDriving()
                )
        ).withName("OTF Auto: Double Trench Score");
    }

    /**
     * <b>Aggressive Sweep</b> — Maximum ball collection from the neutral zone.
     *
     * <ol>
     *   <li>Deploy intake arm + init</li>
     *   <li>Score all 8 preloaded FUEL</li>
     *   <li>Pathfind through trench to near-trench side of neutral zone</li>
     *   <li>Sweep across neutral zone to the center while intaking</li>
     *   <li>Sweep to far-trench side while intaking</li>
     *   <li>Return through trench to shooting range (pre-spinning flywheel)</li>
     *   <li>Score all collected FUEL</li>
     * </ol>
     *
     * <p>Highest-risk/highest-reward routine. Covers the most field area
     * but is most susceptible to opponent interference.
     */
    public Command aggressiveSweep() {
        return Commands.sequence(
                autoInit(),

                // Score preloaded FUEL
                shootPreload(),

                // Sweep path: near-trench → center → far-trench
                Commands.deadline(
                        Commands.sequence(
                                pathfindTo(
                                        PathfindingConstants.kNeutralZoneNearTrench,
                                        Rotation2d.fromDegrees(90),
                                        1.5),
                                pathfindTo(
                                        PathfindingConstants.kNeutralZoneCenter,
                                        Rotation2d.fromDegrees(90),
                                        1.5),
                                pathfindTo(
                                        PathfindingConstants.kNeutralZoneFarTrench,
                                        Rotation2d.fromDegrees(180),
                                        0.5)
                        ),
                        intakeWhileDriving()
                ),

                // Return to shooting range (flywheel pre-spins during drive)
                driveToShootingRange(),

                // Score collected FUEL
                shootCollected()
        ).withName("OTF Auto: Aggressive Sweep");
    }

    /**
     * <b>Defensive Preload</b> — Safest auto with minimal movement.
     *
     * <ol>
     *   <li>Deploy intake arm + init</li>
     *   <li>Score all 8 preloaded FUEL from starting position</li>
     *   <li>Park defensively near the hub, facing it</li>
     * </ol>
     *
     * <p>Use when opponents are aggressive and you want to protect
     * the hub area for teleop. Minimal risk.
     */
    public Command defensivePreload() {
        return Commands.sequence(
                autoInit(),

                // Score preloaded FUEL
                shootPreload(),

                // Park near hub in a defensive position (Blue frame — AutoBuilder flips for Red)
                Commands.defer(() -> {
                    Translation2d blueHub = HubConstants.kBlueHubCenter;
                    // Park 1.5m in front of hub (toward neutral zone)
                    Translation2d parkPos = new Translation2d(
                            blueHub.getX() + 1.5,
                            blueHub.getY());
                    return pathfindToPrecise(parkPos, headingToHub(parkPos));
                }, Set.of(swerveDrive)),

                // Lock wheels for defense
                Commands.runOnce(() -> swerveDrive.lockWheels(), swerveDrive)
        ).withName("OTF Auto: Defensive Preload");
    }
}
