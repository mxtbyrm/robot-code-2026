package frc.robot.constants;

import frc.robot.RobotConfig;

// ==================== BALL PRESENCE ESTIMATION (CURRENT-BASED) ====================
/**
 * Constants for sensorless ball detection using motor current analysis.
 *
 * <p>When a motor runs with balls loaded, friction increases → current rises
 * above the empty-mechanism baseline. The delta reveals ball presence.
 *
 * <p>All thresholds are in AMPS and represent the current ABOVE baseline.
 * Tune these on the real robot by comparing empty vs loaded current readings.
 */
public final class BallDetectionConstants {
    // ---- Voltage compensation ----
    // Baseline currents are calibrated at nominal battery voltage.
    // During a match, voltage sag causes motors to draw more current
    // for the same load. We normalize readings back to this voltage.
    public static final double kNominalCalibrationVoltage = 12.0;

    // ---- Sliding window size for current smoothing ----
    // At 50Hz update rate, 10 samples = 200ms averaging window.
    // Larger = more stable, slower response. Smaller = noisier, faster response.
    public static final int kCurrentHistorySize = 10;

    // ---- Default baselines (used before calibration) ----
    // These are rough estimates — calibrate() overwrites them during health check.
    // TODO: measure on real robot with empty hopper
    public static final double kDefaultSpindexerBaselineCurrent = RobotConfig.kDefaultSpindexerBaselineCurrent;
    public static final double kDefaultFeederBaselineCurrent    = RobotConfig.kDefaultFeederBaselineCurrent;

    // ---- Spindexer hopper detection thresholds ----
    // Delta above baseline needed to detect balls in hopper.
    // kSpindexerLoadedThreshold: above this → hopper has balls (positive detection)
    // kSpindexerEmptyThreshold:  below this → hopper is empty (negative detection)
    // Gap between them = hysteresis band to prevent flickering.
    // TODO: tune on real robot — load 1 ball, measure delta, set thresholds
    public static final double kSpindexerLoadedThreshold = RobotConfig.kSpindexerLoadedThreshold;
    public static final double kSpindexerEmptyThreshold  = RobotConfig.kSpindexerEmptyThreshold;

    // Delta when hopper is completely full (8 preloaded balls).
    // Used to normalize load level to 0.0–1.0 range.
    // TODO: measure with full hopper on real robot
    public static final double kSpindexerFullLoadDelta = RobotConfig.kSpindexerFullLoadDelta;

    // ---- Feeder ball detection threshold ----
    // Current spike when a ball is being actively pushed into flywheels.
    // TODO: tune on real robot — shoot single balls, observe feeder current
    public static final double kFeederBallPresentThreshold = RobotConfig.kFeederBallPresentThreshold;
}
