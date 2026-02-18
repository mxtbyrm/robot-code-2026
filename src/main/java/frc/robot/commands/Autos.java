package frc.robot.commands;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.AutoConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Spindexer;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/**
 * Choreo-based autonomous routines for FRC 2026 REBUILT™.
 *
 * <p>All trajectories are designed in the Choreo GUI and exported as .traj files
 * to {@code src/main/deploy/choreo/}. This class wires trajectory segments to
 * subsystem actions (intake, shoot, idle).
 *
 * <h2>Field Layout (WPILib coordinates — origin at Blue wall):</h2>
 * <pre> 
 *   BLUE ALLIANCE SIDE                                                                        RED ALLIANCE SIDE
 *                     ┌────────────────────────────────────────────────────────────────────┐
 *                     │ [DEPOT]      ┌─TRENCH─┐              ┌─TRENCH─┐         [OUTPOST]  │
 *                     │              │        │              │        │                    │
 *                     │              │ [BUMP] │              │ [BUMP] │                    │
 *                     │              │┌──────┐│              │┌──────┐│                    │
 *                     │              ││ BLUE ││   NEUTRAL    ││ RED  ││                    │ 
 * Blue Driver Station │  [TOWER]     ││ HUB  ││    ZONE      ││ HUB  ││          [TOWER]   │ Red Driver Station
 *                     │              │└──────┘│              │└──────┘│                    │
 *                     │              │ [BUMP] │              │ [BUMP] │                    │
 *                     │              │        │              │        │                    │
 *                     │ [OUTPOST]    └─TRENCH─┘              └─TRENCH─┘          [DEPOT]   │
 *                     └────────────────────────────────────────────────────────────────────┘
 *                         X=0                                                    X=16.54m
 * </pre>
 *
 * <h2>Key Dimensions (from Game Manual):</h2>
 * <ul>
 *   <li>Field: 16.54m × 8.07m (651.2" × 317.7")</li>
 *   <li>AUTO period: <b>20 seconds</b>, both HUBs active</li>
 *   <li>Blue HUB center: (4.03, 4.035) — at robot starting line</li>
 *   <li>Robot starting line: 4.03m (158.6") from alliance wall</li>
 *   <li>Neutral zone FUEL: ~5.2m × 1.8m centered at (8.27, 4.035)</li>
 *   <li>BUMPs: 73"W × 44.4"D × 6.5"H (1.854m × 1.128m × 0.165m) — obstacle</li>
 *   <li>TRENCHes: 65.65"W × 47"D, 22.25" clearance — <b>our path to neutral zone</b></li>
 *   <li>DEPOT: 42"W × 27"D along alliance wall — human player FUEL source</li>
 *   <li>Max preload: 8 FUEL per robot (504 total FUEL per match)</li>
 *   <li>Scoring: 1pt per FUEL in active HUB (AUTO and TELEOP),
 *       15pt per LEVEL 1 TOWER climb (AUTO), 10pt LEVEL 1 (TELEOP),
 *       20pt LEVEL 2 (TELEOP), 30pt LEVEL 3 (TELEOP)</li>
 *   <li>ENERGIZED RP: ≥100 FUEL, SUPERCHARGED RP: ≥360 FUEL,
 *       TRAVERSAL RP: ≥50 TOWER points</li>
 * </ul>
 *
 * <h2>TELEOP Timing (Game Manual Table 6-2):</h2>
 * <ul>
 *   <li>TRANSITION SHIFT: 10s (both HUBs active)</li>
 *   <li>SHIFT 1-4: 25s each (alternating active/inactive)</li>
 *   <li>END GAME: 30s (both HUBs active)</li>
 *   <li>Total TELEOP: 2:20 (140s), Total MATCH: 2:40 (160s)</li>
 * </ul>
 *
 * <h2>Choreo GUI Robot Config:</h2>
 * <pre>
 *   Mass:       ~54 kg          Track width:  0.578m (22.75")
 *   Wheelbase:  0.578m (22.75") Max velocity: 4.5 m/s
 *   Max accel:  3.0 m/s²        Max omega:    6.28 rad/s
 *   Bumper dim: ~0.813m × 0.813m
 * </pre>
 *
 * <h2>Strategy Note:</h2>
 * The alliance scoring MORE FUEL in auto starts with HUB <b>INACTIVE</b>
 * for Teleop Shift 1. Aggressive auto scoring trades teleop start quality —
 * choose routine based on desired auto/teleop tradeoff.
 */
public final class Autos {
    @SuppressWarnings("unused")
    private final SwerveDrive swerveDrive;
    private final Shooter shooter;
    @SuppressWarnings("unused")
    private final Feeder feeder;
    @SuppressWarnings("unused")
    private final Intake intake;
    @SuppressWarnings("unused")
    private final Spindexer spindexer;
    private final Superstructure superstructure;
    @SuppressWarnings("unused")
    private final Vision vision;
    private final AutoFactory autoFactory;

    /** Factory for creating vision-assisted ball chase commands. Injected from RobotContainer. */
    private final Supplier<Command> visionCollectFactory;

    public Autos(SwerveDrive swerveDrive, Shooter shooter, Feeder feeder, Intake intake,
                 Spindexer spindexer, Superstructure superstructure, Vision vision,
                 AutoFactory autoFactory, Supplier<Command> visionCollectFactory) {
        this.swerveDrive = swerveDrive;
        this.shooter = shooter;
        this.feeder = feeder;
        this.intake = intake;
        this.spindexer = spindexer;
        this.superstructure = superstructure;
        this.vision = vision;
        this.autoFactory = autoFactory;
        this.visionCollectFactory = visionCollectFactory;
    }

    // ========================================================================
    //  COMPOSABLE HELPER COMMANDS
    // ========================================================================

    /**
     * Wait for shooter to lock on (flywheel at speed + turret aimed + hood set),
     * then feed FUEL for the given duration via the superstructure.
     */
    private Command waitAndShoot(double feedSeconds) {
        return Commands.sequence(
                Commands.runOnce(() -> shooter.enableTracking()),
                shooter.waitUntilReady().withTimeout(AutoConstants.kShooterLockTimeoutSeconds),
                superstructure.autoShootCommand(feedSeconds),
                Commands.runOnce(() -> shooter.disableTracking())
        ).withTimeout(AutoConstants.kMaxShootTimeoutSeconds + feedSeconds)
         .handleInterrupt(() -> {
             Logger.recordOutput("Auto/Warning", "waitAndShoot timed out");
             superstructure.requestIdle();
             shooter.disableTracking();
         });
    }

    /** Dump all 8 preloaded FUEL. Turret auto-aims at HUB. */
    private Command shootPreload() {
        return waitAndShoot(AutoConstants.kPreloadShootSeconds);
    }

    /** Dump a collected batch of FUEL after a sweep. */
    private Command shootCollected() {
        return waitAndShoot(AutoConstants.kCollectedShootSeconds);
    }

    /**
     * Run intake rollers while a trajectory is active (bind with {@code .whileTrue()}).
     * Arm is deployed once at autonomous start — not managed here.
     * Spindexer does NOT run during intake — only during shooting.
     */
    private Command intakeWhileDriving() {
        return superstructure.intakeRollerCommand();
    }

    /**
     * Vision-assisted ball collection for auto.
     *
     * <p>After the sweep trajectory ends near the ball area, this command
     * uses the intake camera to chase any visible ball. Falls back after
     * a timeout if no ball is detected.
     */
    private Command visionCollect() {
        return Commands.deadline(
                visionCollectFactory.get(),
                intakeWhileDriving()
        ).withTimeout(AutoConstants.kMaxVisionCollectTimeoutSeconds)
         .withName("Choreo: Vision Collect");
    }

    /**
     * Intake + auto-shoot while driving (for shoot-while-moving routines).
     * Uses hysteresis to prevent feeder jitter when isReadyToShoot() flickers
     * at the edge of shooting range. Requires 5 consecutive "ready" cycles to
     * start shooting, and holds shoot mode for at least 10 cycles before
     * dropping back to intake.
     */
    /**
     * Tracks hysteresis state for shoot-while-moving auto commands.
     * Prevents feeder jitter when isReadyToShoot() flickers at range edges.
     */
    private static final class ShootWhileMovingState {
        private static final int kReadyThreshold = 5;
        private static final int kShootHoldMin = 10;

        int readyCycles = 0;
        int shootHoldCycles = 0;
        boolean shooting = false;

        void reset() {
            readyCycles = 0;
            shootHoldCycles = 0;
            shooting = false;
        }
    }

    private Command intakeAndShootWhileDriving() {
        ShootWhileMovingState state = new ShootWhileMovingState();

        return Commands.runEnd(
                () -> {
                    // Tracking must be on for shoot-while-moving
                    if (!shooter.isTrackingEnabled()) shooter.enableTracking();
                    if (state.shooting) {
                        // Currently shooting — hold for minimum duration
                        state.shootHoldCycles++;
                        if (!shooter.isReadyToShoot()
                                && state.shootHoldCycles >= ShootWhileMovingState.kShootHoldMin) {
                            // Lost lock after hold period — drop back to intake
                            state.shooting = false;
                            state.readyCycles = 0;
                            superstructure.requestIntake();
                        } else {
                            superstructure.requestShoot();
                        }
                    } else {
                        // Currently intaking — count consecutive ready cycles
                        if (shooter.isReadyToShoot()) {
                            state.readyCycles++;
                        } else {
                            state.readyCycles = 0;
                        }

                        if (state.readyCycles >= ShootWhileMovingState.kReadyThreshold) {
                            // Stable lock confirmed — switch to shoot
                            state.shooting = true;
                            state.shootHoldCycles = 0;
                            superstructure.requestShoot();
                        } else {
                            superstructure.requestIntake();
                        }
                    }
                },
                () -> {
                    state.reset();
                    shooter.disableTracking();
                    superstructure.requestIdle();
                },
                superstructure
        );
    }

    // ========================================================================
    //  SAFETY (no Choreo dependency — always works)
    // ========================================================================

    /** Does absolutely nothing. Always safe, always compiles, always runs. */
    public Command doNothing() {
        return Commands.none().withName("Do Nothing");
    }

    // ========================================================================
    //  CHOREO ROUTINES
    // ========================================================================

    /**
     * <b>Preload & Park</b> — minimal reliable auto.
     *
     * <p>Drives to shooting range near the HUB, dumps all 8 preloaded FUEL,
     * then drives through the TRENCH into the neutral zone for field position.
     *
     * <p>Expected score: ~8 FUEL = 8 pts.
     *
     * <p><b>Required trajectories ({@code src/main/deploy/choreo/}):</b>
     * <ul>
     *   <li>{@code preload.1} — start position → shooting range near HUB</li>
     *   <li>{@code preload.2} — shooting range → park through TRENCH into neutral zone</li>
     * </ul>
     */
    public AutoRoutine preloadAndPark() {
        AutoRoutine routine = autoFactory.newRoutine("preloadAndPark");

        AutoTrajectory toShoot = routine.trajectory("preload.1");
        AutoTrajectory toPark  = routine.trajectory("preload.2");

        routine.active().onTrue(
                Commands.sequence(toShoot.resetOdometry(),
                        toShoot.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toShoot.done().onTrue(
                Commands.sequence(shootPreload(),
                        toPark.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        return routine;
    }

    /**
     * <b>Center 2-Cycle</b> — shoot preload, sweep neutral zone, shoot again.
     *
     * <p>Starting from center (aligned with HUB): dumps 8 preloaded FUEL,
     * drives through the TRENCH to sweep loose FUEL in the neutral zone with intake
     * running, returns to shooting range, and dumps the collected batch.
     *
     * <p>Expected score: 8 preloaded + 4–8 collected ≈ 12–16 pts.
     *
     * <p><b>Required trajectories:</b>
     * <ul>
     *   <li>{@code center2.1} — center start → shooting range</li>
     *   <li>{@code center2.2} — shooting range → neutral zone sweep path (through TRENCH)</li>
     *   <li>{@code center2.3} — end of sweep → return to shooting range</li>
     * </ul>
     */
    public AutoRoutine centerTwoCycle() {
        AutoRoutine routine = autoFactory.newRoutine("centerTwoCycle");

        AutoTrajectory toShoot  = routine.trajectory("center2.1");
        AutoTrajectory toSweep  = routine.trajectory("center2.2");
        AutoTrajectory toReturn = routine.trajectory("center2.3");

        // Phase 1: Drive to shooting position, dump preload
        routine.active().onTrue(
                Commands.sequence(toShoot.resetOdometry(),
                        toShoot.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toShoot.done().onTrue(
                Commands.sequence(shootPreload(),
                        toSweep.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        // Phase 2: Sweep neutral zone with intake running
        toSweep.active().whileTrue(intakeWhileDriving());
        toSweep.done().onTrue(
                Commands.sequence(visionCollect(),
                        toReturn.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        // Phase 3: Return to shooting range, dump collected batch
        toReturn.done().onTrue(shootCollected());

        return routine;
    }

    /**
     * <b>Left 2-Cycle</b> — shoot preload from left start, sweep, shoot.
     *
     * <p>Same structure as center 2-cycle but from the left starting position.
     * Sweep path targets the left side of the neutral zone near the guardrail/TRENCH.
     *
     * <p><b>Required trajectories:</b>
     * <ul>
     *   <li>{@code left2.1} — left start → shooting range</li>
     *   <li>{@code left2.2} — shooting range → left-side neutral zone sweep</li>
     *   <li>{@code left2.3} — end of sweep → return to shooting range</li>
     * </ul>
     */
    public AutoRoutine leftTwoCycle() {
        AutoRoutine routine = autoFactory.newRoutine("leftTwoCycle");

        AutoTrajectory toShoot  = routine.trajectory("left2.1");
        AutoTrajectory toSweep  = routine.trajectory("left2.2");
        AutoTrajectory toReturn = routine.trajectory("left2.3");

        routine.active().onTrue(
                Commands.sequence(toShoot.resetOdometry(),
                        toShoot.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toShoot.done().onTrue(
                Commands.sequence(shootPreload(),
                        toSweep.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toSweep.active().whileTrue(intakeWhileDriving());
        toSweep.done().onTrue(
                Commands.sequence(visionCollect(),
                        toReturn.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toReturn.done().onTrue(shootCollected());

        return routine;
    }

    /**
     * <b>Right 2-Cycle</b> — shoot preload from right start, sweep, shoot.
     *
     * <p>Same structure as center 2-cycle but from the right starting position.
     * Sweep path targets the right side of the neutral zone near the scoring table.
     *
     * <p><b>Required trajectories:</b>
     * <ul>
     *   <li>{@code right2.1} — right start → shooting range</li>
     *   <li>{@code right2.2} — shooting range → right-side neutral zone sweep</li>
     *   <li>{@code right2.3} — end of sweep → return to shooting range</li>
     * </ul>
     */
    public AutoRoutine rightTwoCycle() {
        AutoRoutine routine = autoFactory.newRoutine("rightTwoCycle");

        AutoTrajectory toShoot  = routine.trajectory("right2.1");
        AutoTrajectory toSweep  = routine.trajectory("right2.2");
        AutoTrajectory toReturn = routine.trajectory("right2.3");

        routine.active().onTrue(
                Commands.sequence(toShoot.resetOdometry(),
                        toShoot.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toShoot.done().onTrue(
                Commands.sequence(shootPreload(),
                        toSweep.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toSweep.active().whileTrue(intakeWhileDriving());
        toSweep.done().onTrue(
                Commands.sequence(visionCollect(),
                        toReturn.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toReturn.done().onTrue(shootCollected());

        return routine;
    }

    /**
     * <b>Center 3-Cycle</b> — maximum scoring auto (tight on 20s timer).
     *
     * <p>Three shooting phases: preload dump → first sweep + dump → second sweep + dump.
     * Requires fast flywheel spin-up, efficient sweep paths through TRENCHes,
     * and reliable intake consistency. This is the most aggressive timed auto.
     *
     * <p>Expected score: 8 preloaded + 4–6 first sweep + 3–5 second sweep ≈ 15–19 pts.
     *
     * <p><b>Required trajectories:</b>
     * <ul>
     *   <li>{@code center3.1} — center start → shooting range</li>
     *   <li>{@code center3.2} — shooting range → first neutral zone sweep</li>
     *   <li>{@code center3.3} — end of first sweep → return to shooting range</li>
     *   <li>{@code center3.4} — shooting range → second neutral zone sweep</li>
     *   <li>{@code center3.5} — end of second sweep → return to shooting range</li>
     * </ul>
     */
    public AutoRoutine centerThreeCycle() {
        AutoRoutine routine = autoFactory.newRoutine("centerThreeCycle");

        AutoTrajectory toShoot1  = routine.trajectory("center3.1");
        AutoTrajectory toSweep1  = routine.trajectory("center3.2");
        AutoTrajectory toReturn1 = routine.trajectory("center3.3");
        AutoTrajectory toSweep2  = routine.trajectory("center3.4");
        AutoTrajectory toReturn2 = routine.trajectory("center3.5");

        // Phase 1: Shoot preload
        routine.active().onTrue(
                Commands.sequence(toShoot1.resetOdometry(),
                        toShoot1.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toShoot1.done().onTrue(
                Commands.sequence(shootPreload(),
                        toSweep1.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        // Phase 2: First sweep + shoot
        toSweep1.active().whileTrue(intakeWhileDriving());
        toSweep1.done().onTrue(
                Commands.sequence(visionCollect(),
                        toReturn1.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toReturn1.done().onTrue(
                Commands.sequence(shootCollected(),
                        toSweep2.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        // Phase 3: Second sweep + shoot
        toSweep2.active().whileTrue(intakeWhileDriving());
        toSweep2.done().onTrue(
                Commands.sequence(visionCollect(),
                        toReturn2.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toReturn2.done().onTrue(shootCollected());

        return routine;
    }

    /**
     * <b>Shoot While Moving</b> — continuous sweep + auto-fire.
     *
     * <p>Instead of stop-shoot-go cycles, the robot drives through FUEL in the
     * neutral zone while staying within shooting range of the HUB. The turret,
     * hood, and flywheel auto-compensate for robot velocity. FUEL is fed into
     * the shooter whenever the system reports ready (in range + locked on).
     *
     * <p>This is the most advanced auto — requires well-tuned shoot-while-moving
     * compensation and a sweep path that stays within effective shooting distance
     * (~2–5m from HUB) the entire time.
     *
     * <p>Expected score: 8 preloaded + continuous collection ≈ 12–20+ pts.
     *
     * <p><b>Required trajectories:</b>
     * <ul>
     *   <li>{@code swm.1} — start → first sweep through neutral zone (staying in HUB range)</li>
     *   <li>{@code swm.2} — loop back for second sweep pass</li>
     * </ul>
     */
    public AutoRoutine shootWhileMoving() {
        AutoRoutine routine = autoFactory.newRoutine("shootWhileMoving");

        AutoTrajectory sweep1 = routine.trajectory("swm.1");
        AutoTrajectory sweep2 = routine.trajectory("swm.2");

        // Start first sweep: intake collects, shooter fires when ready
        routine.active().onTrue(
                Commands.sequence(sweep1.resetOdometry(),
                        sweep1.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds * 2)));

        sweep1.active().whileTrue(intakeAndShootWhileDriving());
        sweep1.done().onTrue(
                sweep2.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds * 2));

        sweep2.active().whileTrue(intakeAndShootWhileDriving());
        sweep2.done().onTrue(
                Commands.runOnce(superstructure::requestIdle, superstructure));

        return routine;
    }

    /**
     * <b>Depot Cycle</b> — use human player FUEL from the DEPOT.
     *
     * <p>Shoot preloaded FUEL, then drive to the DEPOT (42" wide, along alliance
     * wall) to collect FUEL placed by the human player, return to shooting range,
     * and shoot. Useful when neutral zone FUEL is heavily contested or when
     * coordinating with the human player for a controlled feed.
     *
     * <p>The DEPOT is within the ALLIANCE ZONE (behind the starting line), so
     * the robot does not need to cross through a TRENCH for this path.
     *
     * <p>Expected score: 8 preloaded + human-player batch ≈ 12–16 pts.
     *
     * <p><b>Required trajectories:</b>
     * <ul>
     *   <li>{@code depot.1} — start → shooting range near HUB</li>
     *   <li>{@code depot.2} — shooting range → DEPOT (along alliance wall)</li>
     *   <li>{@code depot.3} — DEPOT → return to shooting range</li>
     * </ul>
     */
    public AutoRoutine depotCycle() {
        AutoRoutine routine = autoFactory.newRoutine("depotCycle");

        AutoTrajectory toShoot  = routine.trajectory("depot.1");
        AutoTrajectory toDepot  = routine.trajectory("depot.2");
        AutoTrajectory toReturn = routine.trajectory("depot.3");

        routine.active().onTrue(
                Commands.sequence(toShoot.resetOdometry(),
                        toShoot.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toShoot.done().onTrue(
                Commands.sequence(shootPreload(),
                        toDepot.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        // Intake running while driving through DEPOT to collect human player FUEL
        toDepot.active().whileTrue(intakeWhileDriving());
        toDepot.done().onTrue(
                Commands.sequence(visionCollect(),
                        toReturn.cmd().withTimeout(AutoConstants.kMaxTrajectoryTimeoutSeconds)));

        toReturn.done().onTrue(shootCollected());

        return routine;
    }
}
