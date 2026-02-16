package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.littletonrobotics.junction.Logger;

/**
 * Tracks the HUB's Active/Inactive state during a match based on
 * the ALLIANCE SHIFT rotation mechanic in 2026 REBUILT™.
 *
 * <h2>Rules Summary:</h2>
 * <ul>
 *   <li><b>Active:</b> DMX light bars ON → scored FUEL = 1 point</li>
 *   <li><b>Inactive:</b> DMX light bars OFF → scored FUEL = 0 points</li>
 *   <li>During Teleop ALLIANCE SHIFTS, status alternates on a timer.</li>
 *   <li>The alliance that scored MORE FUEL in Auto starts with HUB
 *       <b>Inactive</b> for Shift 1 (penalty for dominating auto).</li>
 * </ul>
 *
 * <h2>Game Data:</h2>
 * <p>The FMS sends a single character ('R' or 'B') via
 * {@code DriverStation.getGameSpecificMessage()} approximately 3 seconds
 * after Auto ends. This character indicates which alliance's HUB goes
 * <b>inactive first</b> (Shifts 1 & 3). That alliance's HUB is
 * <b>active</b> in Shifts 2 & 4. The data is empty until sent.</p>
 *
 * <h2>Strategic Impact:</h2>
 * Shooting while the HUB is Inactive wastes cycles. The robot should
 * either hold FUEL until Active, or focus on intake/defense during
 * Inactive windows.
 *
 * <p>This tracker estimates the state based on match time and a
 * configurable shift schedule. In competition, the FMS DMX state
 * should be read from NetworkTables if available as the source of truth.
 */
public class HubStateTracker extends SubsystemBase {

    // ==================== SINGLETON ====================
    private static HubStateTracker instance;

    public static HubStateTracker getInstance() {
        if (instance == null) instance = new HubStateTracker();
        return instance;
    }

    /**
     * Inject a custom instance (for unit testing with mocks).
     * Call before any getInstance() usage in tests.
     */
    public static void setInstance(HubStateTracker customInstance) {
        instance = customInstance;
    }

    /** Reset singleton (call in test teardown). */
    public static void resetInstance() {
        instance = null;
    }

    // ==================== MATCH TIMING (Game Manual Table 6-2) ====================
    /**
     * Teleop total duration: 2 minutes 20 seconds = 140 seconds.
     * Match total: 20s AUTO + 140s TELEOP = 160s.
     */
    public static final double kTeleopDurationSeconds = 140.0;

    /**
     * TRANSITION SHIFT: first 10 seconds of TELEOP (timer 2:20 → 2:10).
     * Both ALLIANCE HUBs are ACTIVE during TRANSITION.
     * HUB lights indicate which HUB will be inactive in SHIFT 1.
     */
    public static final double kTransitionDurationSeconds = 10.0;

    /**
     * Duration of each ALLIANCE SHIFT window: 25 seconds.
     * SHIFT 1: 2:10 → 1:45, SHIFT 2: 1:45 → 1:20,
     * SHIFT 3: 1:20 → 0:55, SHIFT 4: 0:55 → 0:30.
     */
    public static final double kShiftDurationSeconds = 25.0;

    /**
     * Number of ALLIANCE SHIFTS in Teleop (SHIFT 1 through SHIFT 4).
     */
    public static final int kTotalShifts = 4;

    /**
     * END GAME: last 30 seconds of TELEOP (timer 0:30 → 0:00).
     * Both ALLIANCE HUBs return to ACTIVE during END GAME.
     */
    public static final double kEndGameDurationSeconds = 30.0;

    /**
     * Elapsed time at which ALLIANCE SHIFTS begin (after TRANSITION).
     */
    public static final double kShiftsStartElapsed = kTransitionDurationSeconds; // 10s

    /**
     * Elapsed time at which END GAME begins.
     * = TRANSITION (10) + 4 shifts × 25s = 110s elapsed.
     */
    public static final double kEndGameStartElapsed =
            kTransitionDurationSeconds + (kTotalShifts * kShiftDurationSeconds); // 110s

    /**
     * FUEL scored in the HUB continues to count for up to 3 seconds after
     * the HUB deactivates (Game Manual Section 6.5). This grace period means
     * shooting right before deactivation is still productive.
     */
    public static final double kDeactivationGracePeriodSeconds = 3.0;

    // ==================== STATE ====================
    public enum HubStatus {
        /** HUB is Active — scored FUEL earns points */
        ACTIVE,
        /** HUB is Inactive — scored FUEL earns 0 points */
        INACTIVE,
        /** Unknown — pre-match or FMS data unavailable */
        UNKNOWN
    }

    private HubStatus currentStatus = HubStatus.UNKNOWN;
    private int currentShiftNumber = 0;

    /**
     * Whether our alliance's HUB goes inactive first (Shifts 1 & 3).
     * Determined automatically from FMS Game Data: the alliance that scored
     * more FUEL in Auto has its HUB go inactive first.
     * <p>True  → our HUB: Inactive in Shifts 1 & 3, Active in Shifts 2 & 4.
     * <p>False → our HUB: Active in Shifts 1 & 3, Inactive in Shifts 2 & 4.
     */
    private boolean weGoInactiveFirst = false;

    /** True once FMS Game Data has been received and parsed. */
    private boolean gameDataReceived = false;

    /** Timestamp when Teleop started (FPGA fallback only). */
    private double teleopStartTime = -1;

    /** True when FMS is attached and providing MatchTime. */
    private boolean fmsTimingAvailable = false;

    private HubStateTracker() {}

    // ==================== MATCH LIFECYCLE ====================

    /** Called when Autonomous starts. Resets state. */
    public void onAutoInit() {
        currentStatus = HubStatus.ACTIVE; // HUB is Active during Autonomous
        currentShiftNumber = 0;
        teleopStartTime = -1;
        gameDataReceived = false;
        weGoInactiveFirst = false;
    }

    /** Called when Teleop starts. Begins shift tracking. */
    public void onTeleopInit() {
        teleopStartTime = Timer.getFPGATimestamp();
        fmsTimingAvailable = DriverStation.isFMSAttached();
        currentShiftNumber = 1;
        // Game Data may not have arrived yet (~3s after Auto ends).
        // Try reading immediately, but periodic() will keep polling.
        pollGameData();
        updateShiftStatus();
    }

    /**
     * Manual override: force the inactive-first flag.
     * Use from the dashboard in practice when FMS is not attached.
     * @param weWonAuto true if our alliance scored more FUEL in Autonomous
     *                  (our HUB goes inactive first)
     */
    public void setWonAutonomous(boolean weWonAuto) {
        this.weGoInactiveFirst = weWonAuto;
        this.gameDataReceived = true; // treat manual input as authoritative
    }

    /**
     * Polls {@code DriverStation.getGameSpecificMessage()} for the FMS Game Data.
     * The data arrives ~3 seconds after Auto ends as a single character:
     *   'R' → Red alliance goes inactive first
     *   'B' → Blue alliance goes inactive first
     *
     * <p>Compares against our own alliance color to determine if WE go inactive first.
     */
    private void pollGameData() {
        if (gameDataReceived) return; // already have it

        String gameData = DriverStation.getGameSpecificMessage();
        if (gameData == null || gameData.isEmpty()) return; // not yet received

        char inactiveFirstAlliance = gameData.charAt(0);
        if (inactiveFirstAlliance != 'R' && inactiveFirstAlliance != 'B') {
            // Corrupt data — ignore
            DriverStation.reportWarning(
                    "[HubStateTracker] Corrupt Game Data: '" + gameData + "'", false);
            return;
        }

        // Determine if it's US
        var ourAlliance = DriverStation.getAlliance();
        if (ourAlliance.isEmpty()) return; // alliance not known yet — retry next cycle

        boolean weAreRed = ourAlliance.get() == DriverStation.Alliance.Red;
        weGoInactiveFirst = (inactiveFirstAlliance == 'R') == weAreRed;

        gameDataReceived = true;
        DriverStation.reportWarning(
                "[HubStateTracker] Game Data received: '" + inactiveFirstAlliance
                + "' — we go inactive first: " + weGoInactiveFirst, false);
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        // Keep polling until Game Data arrives (~3s after Auto ends)
        if (!gameDataReceived) {
            pollGameData();
        }

        if (DriverStation.isTeleopEnabled() && teleopStartTime > 0) {
            updateFromMatchTime();
        } else if (DriverStation.isAutonomousEnabled()) {
            currentStatus = HubStatus.ACTIVE;
        } else {
            currentStatus = HubStatus.UNKNOWN;
        }

        // ---- Telemetry (AdvantageKit only) ----
        double timeUntilShift = getTimeUntilNextShift();
        Logger.recordOutput("HUB/Status", currentStatus.name());
        Logger.recordOutput("HUB/ShiftNumber", currentShiftNumber);
        Logger.recordOutput("HUB/GameDataReceived", gameDataReceived);
        Logger.recordOutput("HUB/WeGoInactiveFirst", weGoInactiveFirst);
        Logger.recordOutput("HUB/ShouldShoot", shouldShoot());
        Logger.recordOutput("HUB/TimeUntilShift", timeUntilShift);
    }

    // ==================== SHIFT LOGIC ====================

    /**
     * Updates HUB state using FMS MatchTime when available (counts down from 140→0),
     * falling back to local FPGA timer if FMS is not attached.
     * FMS MatchTime is authoritative — immune to delayed match starts.
     *
     * <p>TELEOP period structure (Game Manual Table 6-2):
     * <pre>
     *   0–10s elapsed:   TRANSITION SHIFT (both HUBs active)
     *   10–35s elapsed:  SHIFT 1 (one active, one inactive)
     *   35–60s elapsed:  SHIFT 2 (swap)
     *   60–85s elapsed:  SHIFT 3 (swap)
     *   85–110s elapsed: SHIFT 4 (swap)
     *   110–140s elapsed: END GAME (both HUBs active)
     * </pre>
     */
    private void updateFromMatchTime() {
        double elapsed;

        double fmsMatchTime = DriverStation.getMatchTime(); // counts DOWN
        if (fmsTimingAvailable && fmsMatchTime >= 0) {
            // FMS provides time remaining in teleop (140→0).
            // elapsed = total teleop duration - time remaining.
            elapsed = kTeleopDurationSeconds - fmsMatchTime;
        } else {
            // Fallback: local FPGA timer (may drift if match start was delayed)
            elapsed = Timer.getFPGATimestamp() - teleopStartTime;
        }

        // Clamp to prevent negative elapsed from FMS timing quirks
        elapsed = Math.max(0, elapsed);

        if (elapsed < kShiftsStartElapsed) {
            // TRANSITION SHIFT: both HUBs active
            currentStatus = HubStatus.ACTIVE;
            currentShiftNumber = 0;
            return;
        }

        if (elapsed >= kEndGameStartElapsed) {
            // END GAME: both HUBs active
            currentStatus = HubStatus.ACTIVE;
            currentShiftNumber = kTotalShifts + 1; // past all shifts
            return;
        }

        // ALLIANCE SHIFTS: calculate which shift we're in (1-based)
        double shiftElapsed = elapsed - kShiftsStartElapsed;
        currentShiftNumber = (int) (shiftElapsed / kShiftDurationSeconds) + 1;
        currentShiftNumber = Math.min(currentShiftNumber, kTotalShifts);

        updateShiftStatus();
    }

    /**
     * Determines Active/Inactive based on current shift number and auto result.
     *
     * Game Data says: the indicated alliance goes inactive first (Shifts 1 & 3),
     * active in Shifts 2 & 4.
     *
     * If weGoInactiveFirst → Shift 1 = INACTIVE, Shift 2 = ACTIVE, ...
     * If !weGoInactiveFirst → Shift 1 = ACTIVE,   Shift 2 = INACTIVE, ...
     *
     * If Game Data hasn't arrived yet, default to ACTIVE (don't hold fire).
     */
    private void updateShiftStatus() {
        if (!gameDataReceived) {
            // Game Data not yet received — default to shooting
            currentStatus = HubStatus.ACTIVE;
            return;
        }

        boolean oddShift = (currentShiftNumber % 2 == 1);

        if (weGoInactiveFirst) {
            // Our HUB: inactive in odd shifts (1, 3), active in even shifts (2, 4)
            currentStatus = oddShift ? HubStatus.INACTIVE : HubStatus.ACTIVE;
        } else {
            // Our HUB: active in odd shifts (1, 3), inactive in even shifts (2, 4)
            currentStatus = oddShift ? HubStatus.ACTIVE : HubStatus.INACTIVE;
        }
    }

    // ==================== GETTERS ====================

    /** @return current HUB Active/Inactive status */
    public HubStatus getStatus() {
        return currentStatus;
    }

    /** @return true if HUB is Active (scoring yields points) */
    public boolean isHubActive() {
        return currentStatus == HubStatus.ACTIVE;
    }

    /** @return true if FMS Game Data has been received and parsed */
    public boolean isGameDataReceived() {
        return gameDataReceived;
    }

    /** @return current Alliance Shift number (1-based), 0 if not in Teleop */
    public int getCurrentShiftNumber() {
        return currentShiftNumber;
    }

    /**
     * @return seconds until the next period boundary (shift change, end game start, etc.),
     *         or 0 if past all shifts / not in teleop
     */
    public double getTimeUntilNextShift() {
        if (teleopStartTime < 0 || !DriverStation.isTeleopEnabled()) return 0;

        double elapsed;
        double fmsMatchTime = DriverStation.getMatchTime();
        if (fmsTimingAvailable && fmsMatchTime >= 0) {
            elapsed = kTeleopDurationSeconds - fmsMatchTime;
        } else {
            elapsed = Timer.getFPGATimestamp() - teleopStartTime;
        }
        elapsed = Math.max(0, elapsed);

        if (elapsed < kShiftsStartElapsed) {
            // In TRANSITION — next boundary is start of SHIFT 1
            return kShiftsStartElapsed - elapsed;
        }

        if (elapsed >= kEndGameStartElapsed) {
            // In END GAME — no more transitions
            return 0;
        }

        // In ALLIANCE SHIFTS — calculate time until next shift boundary
        double shiftElapsed = elapsed - kShiftsStartElapsed;
        int currentShift = (int) (shiftElapsed / kShiftDurationSeconds) + 1;
        double nextShiftTime = kShiftsStartElapsed + (currentShift * kShiftDurationSeconds);
        double remaining = nextShiftTime - elapsed;
        return Math.max(0, remaining);
    }

    // ==================== STRATEGIC DECISIONS ====================

    /**
     * Returns true if the robot should be actively shooting right now.
     * Takes into account HUB status, time until next shift, and the
     * 3-second deactivation grace period (FUEL scored up to 3s after
     * HUB deactivates still counts — Game Manual Section 6.5).
     *
     * Strategy: Always shoot when Active. When Inactive, shoot if:
     * - A shift to Active is imminent (within 3 seconds), OR
     * - We just entered Inactive (within grace period, FUEL still counts).
     */
    public boolean shouldShoot() {
        if (currentStatus == HubStatus.ACTIVE) return true;
        if (currentStatus == HubStatus.UNKNOWN) return true; // default to shooting
        // Inactive — shoot if shift is imminent (pre-aim + grace period)
        return getTimeUntilNextShift() < kDeactivationGracePeriodSeconds;
    }

    /**
     * Returns true if the robot should prioritize intake/defense instead of shooting.
     * Useful for autonomous strategy and Teleop decision-making.
     */
    public boolean shouldFocusOnIntake() {
        return currentStatus == HubStatus.INACTIVE && getTimeUntilNextShift() > 5.0;
    }
}
