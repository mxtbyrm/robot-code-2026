package frc.robot.constants;

import frc.robot.RobotConfig;

public final class FeederConstants {
    // ==================== MOTOR CAN ID ====================
    public static final int kFeederMotorId = RobotConfig.kFeederMotorId;

    // ==================== BEAM BREAK SENSOR ====================
    // DIO port for the beam break sensor at the feeder exit (before flywheels).
    // This is the ONLY beam break on the robot — used to detect balls leaving
    // the magazine and decrement ball count.
    public static final int kBeamBreakDIOPort = RobotConfig.kBeamBreakDIOPort;

    // ==================== FEEDER CONSTANTS ====================
    // Value set in RobotConfig.java — edit there for new robot deployments.
    public static final double kFeederCurrentLimit = RobotConfig.kFeederCurrentLimit;

    // ==================== STALL / JAM DETECTION ====================
    public static final double kFeederStallCurrentThreshold = 25.0; // Amps — jam detected above this
    public static final double kFeederStallVelocityThreshold = 0.5; // RPS — considered stalled below this
    public static final int kFeederStallCycleThreshold = 10;        // consecutive cycles before jam declared
    public static final double kFeederUnjamReverseDuration = 0.25;  // seconds to reverse when jammed
    public static final double kFeederUnjamReverseSpeed = -0.5;     // duty cycle for unjam reverse

    // ==================== FEEDER SPEED (ratio of shooter) ====================
    // Feeder speed is derived directly from the shooter's target flywheel RPS:
    //   feeder_duty = kFeederSpeedRatio × (targetFlywheelRPS / kMaxFlywheelRPS)
    //
    // No separate interpolation table needed — the shooter already does the
    // distance-based lookup. The feeder and spindexer form a chain:
    //   Shooter RPS → Feeder duty → Spindexer duty
    //
    // SPEED HIERARCHY (CRITICAL — prevents jams):
    //   Shooter flywheel >> Feeder > Spindexer
    //   Feeder = kFeederSpeedRatio × normalized shooter RPS
    //   Spindexer = kSpindexerToFeederRatio × feeder duty (always < feeder)
    //   Flywheel surface speed is always >> feeder surface speed by design
    //   (flywheel runs velocity PID at high RPS, feeder is duty-cycle capped).
    public static final double kFeederSpeedRatio = RobotConfig.kFeederSpeedRatio;
}
