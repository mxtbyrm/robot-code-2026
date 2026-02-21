package frc.robot.sim;

import org.littletonrobotics.junction.Logger;

/**
 * Central simulation coordinator for Robot 2026.
 *
 * <h2>Role:</h2>
 * Owns all per-subsystem physics simulations and runs a ball-flow state machine that
 * models the real game-piece path through the robot:
 * <pre>
 *   Ground → Intake Roller → Hopper (Spindexer) → Feeder → Shooter Flywheel → HUB
 * </pre>
 *
 * <h2>Subsystem sims owned here:</h2>
 * <ul>
 *   <li>{@link ShooterSim}         — Flywheel, Hood, Turret physics
 *   <li>{@link IntakeSim}          — Deploy arm + roller physics
 *   <li>{@link FeederSpindexerSim} — Feeder, Spindexer, and beam-break GPIO
 *   <li>{@link VisionSim}          — PhotonLib camera sim wired to robot pose
 * </ul>
 * Note: {@link SwerveDriveSim} is managed inside {@link frc.robot.subsystems.SwerveDrive}
 * (it is always active in sim and does not need to be called here).
 *
 * <h2>Ball state machine:</h2>
 * Three integer/boolean counters track simulated ball positions:
 * <ul>
 *   <li>{@code ballsInHopper}  — balls currently inside the spindexer hopper
 *   <li>{@code ballInFeeder}   — a ball has reached the feeder exit (beam-break position)
 *   <li>{@code ballsShot}      — cumulative balls scored into the HUB
 * </ul>
 *
 * State transitions are timer-gated so the sim behaves realistically
 * (arm must be down and roller spinning before a ball enters; feeder + flywheel
 * must be running before a shot registers).
 *
 * <h2>Usage:</h2>
 * {@link frc.robot.Robot#simulationInit()} creates a single instance; {@link frc.robot.Robot#simulationPeriodic()} calls {@link #update} each loop.
 */
public class RobotSimulator {

    // ==================== SUBSYSTEM SIMS ====================

    private final ShooterSim          shooterSim;
    private final IntakeSim           intakeSim;
    private final FeederSpindexerSim  feederSpindexerSim;
    private final VisionSim           visionSim;

    // ==================== BALL STATE MACHINE ====================

    /** Balls currently inside the hopper (spindexer bowl). Maximum is {@link #kMaxHopperCapacity}. */
    private int     ballsInHopper = 0;

    /** {@code true} when a ball has been transported to the feeder-exit / beam-break position. */
    private boolean ballInFeeder  = false;

    /** Cumulative balls scored into the HUB since sim started. */
    private int     ballsShot     = 0;

    /** Maximum balls the hopper can hold (matches real robot capacity). */
    private static final int kMaxHopperCapacity = 5;

    // ==================== BALL TRANSPORT TIMING ====================

    /**
     * Seconds of continuous intaking required to add one ball to the hopper.
     * Roughly 1 ball per second at typical intake speeds.
     */
    private static final double kSecondsPerBallIntake    = 1.0;

    /**
     * Seconds for the spindexer/feeder to transport a ball from the hopper
     * to the beam-break position once both motors are running.
     */
    private static final double kHopperToFeederSeconds   = 0.35;

    /**
     * Seconds from feeder+flywheel running until the ball exits the shooter.
     * Represents the physical transit time through the feeder and across the flywheel gap.
     */
    private static final double kShotDurationSeconds     = 0.20;

    // ==================== TIMING ACCUMULATORS ====================

    /** Accumulated time of intake deploying + roller running (resets on intake stopping). */
    private double intakeAccumulatedTime = 0.0;

    /** Accumulated time of spindexer + feeder running with a ball in the hopper. */
    private double feedAccumulatedTime   = 0.0;

    /** Accumulated time of flywheel-at-speed + feeder running with a ball in the feeder. */
    private double shotAccumulatedTime   = 0.0;

    // ==================== CONSTRUCTOR ====================

    /**
     * Creates and initialises all subsystem physics simulations.
     *
     * <p><b>Must be called after all subsystems have been initialised</b>
     * (i.e., after {@code new RobotContainer()}).
     * Each subsystem sim fetches its motor {@link com.ctre.phoenix6.sim.TalonFXSimState}
     * from the subsystem singleton, so the singletons must exist first.
     */
    public RobotSimulator() {
        shooterSim         = new ShooterSim();
        intakeSim          = new IntakeSim();
        feederSpindexerSim = new FeederSpindexerSim();
        visionSim          = new VisionSim();
    }

    // ==================== UPDATE ====================

    /**
     * Advances all subsystem physics and the ball state machine by one loop cycle.
     *
     * <p>Call once per robot periodic loop from
     * {@link frc.robot.Robot#simulationPeriodic()}.
     *
     * @param dtSeconds loop period in seconds (typically 0.02)
     */
    public void update(double dtSeconds) {
        // Step all physics models forward one time slice.
        shooterSim.update(dtSeconds);
        intakeSim.update(dtSeconds);
        feederSpindexerSim.update(dtSeconds);
        visionSim.update();

        // Run ball transit logic based on the fresh physics states.
        updateBallStateMachine(dtSeconds);

        // Drive the beam-break GPIO to match ball position.
        // The Feeder subsystem reads this DIO to know if a ball is present.
        feederSpindexerSim.setBeamBreakTripped(ballInFeeder);

        logTelemetry();
    }

    // ==================== BALL STATE MACHINE ====================

    /**
     * Simulates ball flow through the robot based on the current mechanical states.
     *
     * <h2>Stage 1 — Intake → Hopper:</h2>
     * A ball is added to the hopper every {@link #kSecondsPerBallIntake} seconds
     * while the deploy arm is down ({@link IntakeSim#isDeployed()}) AND the roller
     * is spinning ({@link IntakeSim#isRollerRunning()}) AND the hopper is not full.
     *
     * <h2>Stage 2 — Hopper → Feeder:</h2>
     * A ball transitions from the hopper to the beam-break position after
     * {@link #kHopperToFeederSeconds} of continuous spindexer + feeder running
     * with at least one ball in the hopper and no ball already at the feeder exit.
     *
     * <h2>Stage 3 — Feeder → Shot:</h2>
     * A ball is counted as scored after {@link #kShotDurationSeconds} of the feeder
     * running and the flywheel being at (or near) target speed while a ball sits at
     * the feeder exit.
     */
    private void updateBallStateMachine(double dt) {

        // ---- Stage 1: Ground → Hopper via Intake ----
        boolean intaking = intakeSim.isDeployed()
                        && intakeSim.isRollerRunning()
                        && ballsInHopper < kMaxHopperCapacity;

        if (intaking) {
            intakeAccumulatedTime += dt;
            if (intakeAccumulatedTime >= kSecondsPerBallIntake) {
                ballsInHopper++;
                intakeAccumulatedTime = 0.0;
            }
        } else {
            // Reset accumulator the moment intake conditions break.
            // Partial-time is discarded — you must hold the arm down continuously.
            intakeAccumulatedTime = 0.0;
        }

        // ---- Stage 2: Hopper → Feeder (beam-break position) ----
        boolean canFeed = !ballInFeeder
                       && ballsInHopper > 0
                       && feederSpindexerSim.isSpindexerRunning()
                       && feederSpindexerSim.isFeederRunning();

        if (canFeed) {
            feedAccumulatedTime += dt;
            if (feedAccumulatedTime >= kHopperToFeederSeconds) {
                ballsInHopper--;
                ballInFeeder = true;
                feedAccumulatedTime = 0.0;
            }
        } else {
            feedAccumulatedTime = 0.0;
        }

        // ---- Stage 3: Feeder exit → HUB (scored) ----
        boolean shooting = ballInFeeder
                        && feederSpindexerSim.isFeederRunning()
                        && shooterSim.isFlywheelApproxAtSpeed();

        if (shooting) {
            shotAccumulatedTime += dt;
            if (shotAccumulatedTime >= kShotDurationSeconds) {
                ballInFeeder = false;
                ballsShot++;
                shotAccumulatedTime = 0.0;
            }
        } else {
            // Reset: if the feeder stops or flywheel drops below idle,
            // the ball stays at the beam break — it doesn't fall back.
            if (!ballInFeeder) {
                shotAccumulatedTime = 0.0;
            }
        }
    }

    // ==================== TELEMETRY ====================

    private void logTelemetry() {
        Logger.recordOutput("Sim/Balls/InHopper",    ballsInHopper);
        Logger.recordOutput("Sim/Balls/InFeeder",    ballInFeeder);
        Logger.recordOutput("Sim/Balls/TotalShot",   ballsShot);
        Logger.recordOutput("Sim/Balls/TotalOnRobot",
                ballsInHopper + (ballInFeeder ? 1 : 0));
        Logger.recordOutput("Sim/Balls/IntakeTimer", intakeAccumulatedTime);
        Logger.recordOutput("Sim/Balls/FeedTimer",   feedAccumulatedTime);
        Logger.recordOutput("Sim/Balls/ShotTimer",   shotAccumulatedTime);
    }

    // ==================== ACCESSORS ====================

    /** @return number of balls currently in the hopper (0–{@link #kMaxHopperCapacity}). */
    public int getBallsInHopper() { return ballsInHopper; }

    /** @return {@code true} when a ball is at the feeder-exit beam-break position. */
    public boolean isBallInFeeder() { return ballInFeeder; }

    /** @return cumulative balls scored into the HUB since sim started. */
    public int getBallsShot() { return ballsShot; }

    /** @return the {@link VisionSim} (for debug field overlay in SmartDashboard). */
    public VisionSim getVisionSim() { return visionSim; }
}
