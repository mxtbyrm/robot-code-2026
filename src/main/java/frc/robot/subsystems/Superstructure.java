package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.BallPresenceEstimator;
import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SpindexerConstants;
import frc.robot.HubStateTracker;

import org.littletonrobotics.junction.Logger;

/**
 * Superstructure — the brain that coordinates Intake, Spindexer, Feeder, and Shooter
 * as a unified state machine.
 *
 * <h2>Why this matters for Einstein:</h2>
 * Without a superstructure, parallel commands can conflict (e.g., feeder runs while
 * intake is unjamming, or shooter is not ready). The superstructure guarantees that
 * only one action happens at a time and transitions are safe.
 *
 * <h2>States:</h2>
 * <pre>
 * IDLE ──────────► INTAKING                         SHOOTING
 *   ▲                │                                  │
 *   │                ▼                                  │
 *   │            OUTTAKING / UNJAMMING                  │
 *   │                                                   │
 *   └───────────────────────────────────────────────────┘
 * </pre>
 *
 * <h2>Ball counting:</h2>
 * Tracks number of balls shot. Only counts balls exiting the magazine (feeder beam break).
 * No intake counting (balls too small/fast, no intake sensor).
 */
public class Superstructure extends SubsystemBase {

    // ==================== SINGLETON ====================
    private static Superstructure instance;

    public static void initialize() {
        if (instance != null) throw new IllegalStateException("Superstructure already initialized.");
        instance = new Superstructure();
    }

    public static Superstructure getInstance() {
        if (instance == null) throw new IllegalStateException("Superstructure not initialized. Call initialize() first.");
        return instance;
    }

    public static void resetInstance() { instance = null; }

    // ==================== SUBSYSTEM REFERENCES ====================
    private final Intake intake;
    private final Spindexer spindexer;
    private final Feeder feeder;
    private final Shooter shooter;
    private final Vision vision;

    // ==================== STATE MACHINE ====================
    public enum SuperState {
        /** Nothing running. Flywheel pre-spinning, turret tracking. */
        IDLE,
        /** Intake roller running (arm must be deployed separately). Spindexer does NOT run. */
        INTAKING,

        /** Pre-feed reverse: briefly reverses feeder + spindexer to create
         *  ball clearance before feeding. Prevents jams. Auto-transitions to SHOOTING. */
        PRE_FEED_REVERSE,
        /** Feed requested: feeder + spindexer run when shooter is ready. */
        SHOOTING,
        /** Alliance zone dump: fixed shot to lob FUEL into alliance zone. */
        SHOOTING_ALLIANCE,
        /** Simultaneous intake + shooting: intake rollers run while feeder+spindexer feed shooter. */
        SHOOTING_WHILE_INTAKING,
        /** Outtaking: intake deploys + reverses, spindexer reverses. */
        OUTTAKING,
        /** All motors reverse to clear a jam. */
        UNJAMMING,
        /** Disabled / emergency stop. */
        DISABLED
    }

    private SuperState currentState = SuperState.IDLE;
    private SuperState requestedState = SuperState.IDLE;

    // ==================== BALL COUNTING ====================
    /** Balls shot counter — tracked by the feeder beam break only. */
    private int ballsShotCount = 0;

    // ==================== INTERLOCK FLAGS ====================

    /** Tracks whether the intake arm is currently deployed (toggled by operator). */
    private boolean intakeArmDeployed = false;

    // ==================== PRE-FEED REVERSE ====================
    /** Timestamp when pre-feed reverse started. */
    private double preFeedReverseStartTime = 0;
    /** Which shooting state to transition to after pre-feed reverse completes. */
    private SuperState postReverseShotState = SuperState.SHOOTING;

    // ==================== ENDGAME DUMP ====================
    /** True when endgame dump mode is active (auto-shoot remaining FUEL). */
    private boolean endgameDumpActive = false;
    /** True when operator manually cancelled endgame dump — prevents re-activation. */
    private boolean endgameDumpCancelled = false;

    private Superstructure() {
        this.intake = Intake.getInstance();
        this.spindexer = Spindexer.getInstance();
        this.feeder = Feeder.getInstance();
        this.shooter = Shooter.getInstance();
        this.vision = Vision.getInstance();
    }

    @Override
    public void periodic() {
        // ---- Auto-disable when robot is disabled (match end, E-stop) ----
        if (DriverStation.isDisabled() && currentState != SuperState.DISABLED) {
            requestedState = SuperState.DISABLED;
        }

        // ---- Transition to requested state ----
        if (requestedState != currentState) {
            transitionTo(requestedState);
        }

        // ---- Ball shot counting: beam break between feeder & shooter ----
        // This is the ONLY ball sensor. Counts balls exiting the magazine.
        if (feeder.didBallPassThisCycle()) {
            onBallShot();
        }

        // ---- Execute current state ----
        switch (currentState) {
            case IDLE:
                executeIdle();
                break;
            case INTAKING:
                executeIntaking();
                break;

            case PRE_FEED_REVERSE:
                executePreFeedReverse();
                break;
            case SHOOTING:
                executeShooting();
                break;
            case SHOOTING_ALLIANCE:
                executeShootingAlliance();
                break;
            case SHOOTING_WHILE_INTAKING:
                executeShootingWhileIntaking();
                break;
            case OUTTAKING:
                executeOuttaking();
                break;
            case UNJAMMING:
                executeUnjamming();
                break;
            case DISABLED:
                executeDisabled();
                break;
        }

        // ---- Endgame auto-dump ----
        // In the last N seconds of teleop, automatically enable tracking + shoot
        // to dump all remaining FUEL regardless of Hub active/inactive state.
        // Operator can cancel via emergency stop (START) or idle (BACK) — once
        // cancelled, endgame dump will NOT re-activate for the rest of the match.
        if (DriverStation.isTeleopEnabled()) {
            double matchTime = DriverStation.getMatchTime(); // counts down
            if (matchTime > 0 && matchTime <= ShooterConstants.kEndgameDumpTimeSeconds
                    && !endgameDumpCancelled
                    && currentState != SuperState.DISABLED) {
                if (!endgameDumpActive) {
                    endgameDumpActive = true;
                    shooter.enableTracking();
                    requestedState = SuperState.SHOOTING;
                    DriverStation.reportWarning(
                            "[Superstructure] ENDGAME DUMP activated — dumping all remaining FUEL", false);
                }
                // Do NOT force state every cycle — only set it once on activation.
                // Operator retains full control after activation.
            }
        } else {
            endgameDumpActive = false;
            endgameDumpCancelled = false;
        }

        // ---- Vision distance validation (when shooting) ----
        if (vision != null && (currentState == SuperState.SHOOTING || currentState == SuperState.SHOOTING_WHILE_INTAKING) && shooter.isTrackingEnabled()) {
            vision.validateDistance(shooter.getDistanceToHub());
        }

        // ---- Ball presence estimation (current-based) ----
        BallPresenceEstimator.getInstance().update(
                spindexer.getCurrent(), feeder.getCurrent(),
                spindexer.isRunning(), feeder.isFeeding());

        // ---- Telemetry (AdvantageKit only) ----
        Logger.recordOutput("Super/State", currentState.name());
        Logger.recordOutput("Super/BallsShot", ballsShotCount);
        Logger.recordOutput("Super/ShooterReady", shooter.isReadyToShoot());
        Logger.recordOutput("Super/InRange", shooter.isInShootingRange());
        Logger.recordOutput("Super/IntakeArmDeployed", intakeArmDeployed);
        Logger.recordOutput("Super/EndgameDump", endgameDumpActive);
        Logger.recordOutput("Super/HopperEmpty", BallPresenceEstimator.getInstance().isHopperEmpty());
        Logger.recordOutput("Super/HopperLoad", BallPresenceEstimator.getInstance().getHopperLoadLevel());
    }

    // ==================== STATE TRANSITIONS ====================

    private void transitionTo(SuperState newState) {
        // Exit current state
        switch (currentState) {
            case INTAKING:
                intake.stopRoller(); // only stop roller — arm stays where it is
                break;
            case PRE_FEED_REVERSE:
            case SHOOTING:
            case SHOOTING_ALLIANCE:
                feeder.cancelFeed();
                feeder.stop();
                spindexer.stop();
                break;
            case SHOOTING_WHILE_INTAKING:
                intake.stopRoller();
                feeder.cancelFeed();
                feeder.stop();
                spindexer.stop();
                break;
            case OUTTAKING:
                intake.stopRoller(); // only stop roller — arm stays where it is
                spindexer.stop();
                break;
            case UNJAMMING:
                intake.stopRoller();
                spindexer.stop();
                feeder.stop();
                break;
            default:
                break;
        }

        currentState = newState;

        // Enter new state
        switch (newState) {
            case INTAKING:
                if (intakeArmDeployed) {
                    intake.runRollerIntake();
                }
                break;
            case PRE_FEED_REVERSE:
                // Brief slow reverse to create ball clearance before feeding
                feeder.setPreFeedReverse(ShooterConstants.kPreFeedReverseFeederSpeed);
                spindexer.setSpeed(ShooterConstants.kPreFeedReverseSpindexerSpeed);
                preFeedReverseStartTime = Timer.getFPGATimestamp();
                break;
            case SHOOTING:
                feeder.requestFeed();
                spindexer.setSpeed(computeSpindexerFeedSpeed());
                break;
            case SHOOTING_ALLIANCE:
                feeder.requestFeed();
                spindexer.setSpeed(computeSpindexerFeedSpeed());
                break;
            case SHOOTING_WHILE_INTAKING:
                if (intakeArmDeployed) {
                    intake.runRollerIntake();
                }
                feeder.requestFeed();
                spindexer.setSpeed(computeSpindexerFeedSpeed());
                break;
            case OUTTAKING:
                if (intakeArmDeployed) {
                    intake.runRollerOuttake();
                }
                spindexer.reverse();
                break;
            case UNJAMMING:
                intake.runRollerOuttake();
                spindexer.reverse();
                feeder.reverse();
                break;
            case IDLE:
                spindexer.stop();
                feeder.cancelFeed();
                break;
            case DISABLED:
                intake.stopAll();
                spindexer.stop();
                feeder.stop();
                break;
        }
    }

    // ==================== STATE EXECUTION ====================

    private void executeIdle() {
        // Shooter is always auto-aiming in its own periodic()
        // Nothing else to do in idle
    }

    private void executeIntaking() {
        // Rollers only run when intake arm is deployed past the bumpers.
        // Running rollers while stowed would spin against the frame uselessly.
        // Continuously checked so arm toggle mid-intake responds immediately.
        if (intakeArmDeployed) {
            intake.runRollerIntake();
        } else {
            intake.stopRoller();
        }
    }

    /**
     * Pre-feed reverse: runs feeder + spindexer backward at low speed
     * for a short duration to create ball clearance before shooting.
     * Automatically transitions to the target shooting state when done.
     */
    private void executePreFeedReverse() {
        double elapsed = Timer.getFPGATimestamp() - preFeedReverseStartTime;
        if (elapsed >= ShooterConstants.kPreFeedReverseDurationSeconds) {
            // Reverse complete — transition to the target shooting state.
            // Update requestedState so the normal transition flow handles it.
            requestedState = postReverseShotState;
        }
        // Otherwise keep reversing (motor commands were set in transitionTo entry)
    }

    private void executeShooting() {
        // Gate feeding on HubStateTracker — don't waste FUEL when HUB is inactive.
        // shouldShoot() already handles the 3s grace period: if a shift just happened,
        // FUEL scored within 3s still counts (Game Manual §6.5), so we keep feeding.
        // During deep-inactive windows (>3s from any shift), feeder is paused but
        // flywheel + turret keep tracking so we're instantly ready when HUB reactivates.
        HubStateTracker hub = HubStateTracker.getInstance();
        boolean hubAllowsShoot = hub.shouldShoot();

        if (hubAllowsShoot) {
            // HUB is active or within grace period — feed normally
            if (!feeder.isFeedRequested()) {
                feeder.requestFeed();
            }
            // Update spindexer speed every cycle (tracks shooter RPS dynamically)
            spindexer.setSpeed(computeSpindexerFeedSpeed());
        } else {
            // HUB is inactive and outside grace period — pause feeding, keep tracking
            if (feeder.isFeedRequested()) {
                feeder.cancelFeed();
                spindexer.stop();
            }
        }

        // Telemetry for driver awareness
        Logger.recordOutput("Super/HubActive", hub.isHubActive());
        Logger.recordOutput("Super/HubShouldShoot", hubAllowsShoot);
        Logger.recordOutput("Super/HubNextShift", hub.getTimeUntilNextShift());
    }

    private void executeShootingWhileIntaking() {
        // Intake roller control — same as executeIntaking()
        if (intakeArmDeployed) {
            intake.runRollerIntake();
        } else {
            intake.stopRoller();
        }

        // Feeding logic — same as executeShooting()
        HubStateTracker hub = HubStateTracker.getInstance();
        boolean hubAllowsShoot = hub.shouldShoot();

        if (hubAllowsShoot) {
            if (!feeder.isFeedRequested()) {
                feeder.requestFeed();
            }
            spindexer.setSpeed(computeSpindexerFeedSpeed());
        } else {
            if (feeder.isFeedRequested()) {
                feeder.cancelFeed();
                spindexer.stop();
            }
        }

        Logger.recordOutput("Super/HubActive", hub.isHubActive());
        Logger.recordOutput("Super/HubShouldShoot", hubAllowsShoot);
        Logger.recordOutput("Super/HubNextShift", hub.getTimeUntilNextShift());
    }

    private void executeShootingAlliance() {
        // Alliance zone dump — shooter uses fixed parameters (no auto-aim).
        // Feeder and spindexer are running (set in transitionTo).
        // Feeder gates on shooter.isReadyToShoot() which checks fixed setpoints.
        // Update spindexer speed each cycle to track shooter RPS
        spindexer.setSpeed(computeSpindexerFeedSpeed());
    }

    /**
     * Compute spindexer feed speed as a fraction of the feeder's current speed.
     * Chain: Shooter RPS → Feeder duty → Spindexer duty.
     * Speed hierarchy is automatically guaranteed since ratio < 1.0.
     */
    private double computeSpindexerFeedSpeed() {
        double feederSpeed = feeder.getTargetSpeed();
        return Math.min(
                SpindexerConstants.kSpindexerToFeederRatio * feederSpeed,
                1.0);
    }

    private void executeOuttaking() {
        // Rollers only run when intake arm is deployed
        if (intakeArmDeployed) {
            intake.runRollerOuttake();
        } else {
            intake.stopRoller();
        }
    }

    private void executeUnjamming() {
        // Runs until the driver releases the button
    }

    private void executeDisabled() {
        // Nothing runs
    }

    // ==================== REQUEST METHODS (called by commands/buttons) ====================

    /**
     * Toggle intake arm deploy/stow. Press once to deploy, press again to stow.
     * Independent of roller state — arm position is controlled separately.
     */
    public void toggleIntakeArm() {
        if (currentState == SuperState.DISABLED) return;
        if (intakeArmDeployed) {
            intake.stow();
            intakeArmDeployed = false;
        } else {
            intake.deploy();
            intakeArmDeployed = true;
        }
    }

    /**
     * Deploy intake arm (non-toggle). Used at autonomous start to deploy once.
     * Teleop arm control remains via toggleIntakeArm() on operator A button.
     */
    public void deployIntakeArm() {
        if (currentState != SuperState.DISABLED) {
            intake.deploy();
            intakeArmDeployed = true;
        }
    }

    /** @return true if intake arm is currently deployed */
    public boolean isIntakeArmDeployed() {
        return intakeArmDeployed;
    }

    /** Request intake mode. If currently shooting, transitions to SHOOTING_WHILE_INTAKING. */
    public void requestIntake() {
        if (currentState != SuperState.DISABLED) {
            if (currentState == SuperState.SHOOTING || currentState == SuperState.SHOOTING_WHILE_INTAKING
                    || currentState == SuperState.PRE_FEED_REVERSE) {
                requestedState = SuperState.SHOOTING_WHILE_INTAKING;
            } else {
                requestedState = SuperState.INTAKING;
            }
        }
    }

    /** Request shooting mode. Goes through pre-feed reverse first to prevent jams.
     *  If currently intaking, transitions to SHOOTING_WHILE_INTAKING to keep intake running. */
    public void requestShoot() {
        if (currentState != SuperState.DISABLED) {
            // Skip pre-feed reverse if already shooting or in pre-feed reverse
            if (currentState == SuperState.SHOOTING || currentState == SuperState.SHOOTING_WHILE_INTAKING
                    || currentState == SuperState.PRE_FEED_REVERSE) {
                // Keep combined state if already in it
                if (currentState == SuperState.SHOOTING_WHILE_INTAKING) {
                    requestedState = SuperState.SHOOTING_WHILE_INTAKING;
                } else {
                    requestedState = SuperState.SHOOTING;
                }
            } else if (currentState == SuperState.INTAKING) {
                // Currently intaking — shoot while keeping intake running
                postReverseShotState = SuperState.SHOOTING_WHILE_INTAKING;
                requestedState = SuperState.PRE_FEED_REVERSE;
            } else {
                postReverseShotState = SuperState.SHOOTING;
                requestedState = SuperState.PRE_FEED_REVERSE;
            }
        }
    }

    /** Request alliance zone dump mode. Goes through pre-feed reverse first. */
    public void requestShootAlliance() {
        if (currentState != SuperState.DISABLED) {
            if (currentState == SuperState.SHOOTING_ALLIANCE || currentState == SuperState.PRE_FEED_REVERSE) {
                requestedState = SuperState.SHOOTING_ALLIANCE;
            } else {
                postReverseShotState = SuperState.SHOOTING_ALLIANCE;
                requestedState = SuperState.PRE_FEED_REVERSE;
            }
        }
    }

    /** Request outtake mode (eject balls). */
    public void requestOuttake() {
        if (currentState != SuperState.DISABLED) {
            requestedState = SuperState.OUTTAKING;
        }
    }

    /** Request unjam mode (reverse everything). */
    public void requestUnjam() {
        if (currentState != SuperState.DISABLED) {
            requestedState = SuperState.UNJAMMING;
        }
    }

    /** Stop shooting but keep intaking if in combined state. Otherwise go to idle. */
    public void requestStopShooting() {
        if (currentState == SuperState.SHOOTING_WHILE_INTAKING) {
            requestedState = SuperState.INTAKING;
        } else {
            requestIdle();
        }
    }

    /** Stop intaking but keep shooting if in combined state. Otherwise go to idle. */
    public void requestStopIntaking() {
        if (currentState == SuperState.SHOOTING_WHILE_INTAKING) {
            requestedState = SuperState.SHOOTING;
        } else {
            requestIdle();
        }
    }

    /** Return to idle. Cancels endgame dump if active. */
    public void requestIdle() {
        if (endgameDumpActive) {
            endgameDumpCancelled = true;
            endgameDumpActive = false;
            shooter.disableTracking();
            DriverStation.reportWarning(
                    "[Superstructure] ENDGAME DUMP cancelled by operator", false);
        }
        requestedState = SuperState.IDLE;
    }

    /** Emergency stop all mechanisms. Cancels endgame dump if active. */
    public void requestDisable() {
        if (endgameDumpActive) {
            endgameDumpCancelled = true;
            endgameDumpActive = false;
            shooter.disableTracking();
            DriverStation.reportWarning(
                    "[Superstructure] ENDGAME DUMP cancelled by emergency stop", false);
        }
        requestedState = SuperState.DISABLED;
    }

    // ==================== BALL COUNTING ====================

    /**
     * Called automatically each cycle when the feeder beam break detects
     * a ball passing through to the shooter flywheels.
     * The beam break is between feeder and shooter — a triggered beam break
     * means the ball is definitely being shot.
     */
    private void onBallShot() {
        ballsShotCount++;
        // Notify the shooter so it starts the flywheel recovery cooldown
        shooter.notifyBallShot();
    }

    /** @return total balls shot this match (from feeder beam break) */
    public int getBallsShotCount() {
        return ballsShotCount;
    }

    /** Reset shot counter (call at match start). */
    public void resetBallsShotCount() {
        ballsShotCount = 0;
    }

    // ==================== GETTERS ====================

    public SuperState getCurrentState() {
        return currentState;
    }

    public boolean isShooting() {
        return currentState == SuperState.SHOOTING || currentState == SuperState.SHOOTING_ALLIANCE
                || currentState == SuperState.PRE_FEED_REVERSE || currentState == SuperState.SHOOTING_WHILE_INTAKING;
    }

    public boolean isIntaking() {
        return currentState == SuperState.INTAKING || currentState == SuperState.SHOOTING_WHILE_INTAKING;
    }

    // ==================== COMMAND FACTORIES ====================

    /**
     * Teleop intake roller command — hold to run rollers + spindexer, release stops.
     * Arm position is controlled separately via toggleIntakeArm().
     * Use with {@code whileTrue()}.
     */
    public Command intakeRollerCommand() {
        return Commands.startEnd(
                this::requestIntake,
                this::requestStopIntaking,
                this, intake
        ).withName("Super: Intake Rollers");
    }

    /**
     * Teleop shoot command — hold to feed, release returns to idle.
     * Use with {@code whileTrue()}.
     * Requires intake, spindexer, feeder to prevent conflicting commands.
     */
    public Command shootCommand() {
        return Commands.startEnd(
                this::requestShoot,
                this::requestStopShooting,
                this, spindexer, feeder
        ).withName("Super: Shoot");
    }

    /**
     * Alliance zone dump command — hold to lob FUEL into alliance zone with
     * fixed shooter parameters (no auto-aim). Release returns to idle.
     */
    public Command shootAllianceCommand() {
        return Commands.startEnd(
                this::requestShootAlliance,
                this::requestIdle,
                this, intake, spindexer, feeder
        ).withName("Super: Shoot Alliance Zone");
    }

    /**
     * Teleop outtake command — hold to eject, release returns to idle.
     * Requires intake, spindexer, feeder to prevent conflicting commands.
     */
    public Command outtakeCommand() {
        return Commands.startEnd(
                this::requestOuttake,
                this::requestIdle,
                this, intake, spindexer, feeder
        ).withName("Super: Outtake");
    }

    /**
     * Unjam command — hold to reverse everything, release returns to idle.
     * Requires intake, spindexer, feeder to prevent conflicting commands.
     */
    public Command unjamCommand() {
        return Commands.startEnd(
                this::requestUnjam,
                this::requestIdle,
                this, intake, spindexer, feeder
        ).withName("Super: Unjam");
    }

    /**
     * Auto: run intake rollers for a duration, then idle.
     * Arm is deployed once at autonomous start — not managed here.
     * Spindexer does NOT run during intake — only during shooting.
     *
     * @param timeoutSeconds how long to run the intake
     */
    public Command autoIntakeCommand(double timeoutSeconds) {
        return Commands.sequence(
                Commands.runOnce(this::requestIntake, this, intake, spindexer, feeder),
                Commands.waitSeconds(timeoutSeconds),
                Commands.runOnce(this::requestIdle, this, intake, spindexer, feeder)
        ).withName("Super: Auto Intake");
    }

    /**
     * Auto: shoot (feeder + spindexer) for a duration then return to idle.
     * No ball counting — just feeds for the timeout.
     */
    public Command autoShootCommand(double timeoutSeconds) {
        return Commands.sequence(
                Commands.runOnce(this::requestShoot, this, intake, spindexer, feeder),
                Commands.waitSeconds(timeoutSeconds),
                Commands.runOnce(this::requestIdle, this, intake, spindexer, feeder)
        ).withName("Super: Auto Shoot");
    }
}
