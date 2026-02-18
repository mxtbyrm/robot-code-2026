package frc.robot.constants;

import frc.robot.RobotConfig;

public final class SpindexerConstants {
    // ==================== MOTOR CAN ID ====================
    public static final int kSpindexerMotorId = RobotConfig.kSpindexerMotorId;

    // ==================== SPINDEXER CONSTANTS ====================
    // The spindexer is a rotating platform / indexer that moves balls
    // from the intake hopper toward the feeder (which feeds the shooter).
    // It spins continuously at a set speed while intaking or shooting.
    //
    // SPEED HIERARCHY (CRITICAL — prevents jams):
    //   Shooter flywheel >> Feeder > Spindexer > Intake roller (when feeding)
    //   Each stage must pull balls faster than the previous stage pushes them.
    //   If a downstream stage is slower, balls pile up and jam.
    public static final double kSpindexerIntakeSpeed = RobotConfig.kSpindexerIntakeSpeed;  // slow spin while collecting
    // Feed speed is now computed dynamically as a fraction of the FEEDER speed:
    //   spindexer_duty = kSpindexerToFeederRatio × feeder_duty
    // This creates a chain: Shooter → Feeder → Spindexer
    // The hierarchy is automatically guaranteed as long as ratio < 1.0.
    public static final double kSpindexerToFeederRatio = RobotConfig.kSpindexerToFeederRatio;  // spindexer runs at 65% of feeder speed
    public static final double kSpindexerReverseSpeed = -0.3; // unjam / reverse
    // Value set in RobotConfig.java — edit there for new robot deployments.
    public static final double kSpindexerCurrentLimit = RobotConfig.kSpindexerCurrentLimit;

    // ==================== STALL / JAM DETECTION ====================
    public static final double kSpindexerStallCurrentThreshold = 25.0;
    public static final double kSpindexerStallVelocityThreshold = 0.3;
    public static final int kSpindexerStallCycleThreshold = 15;
    public static final double kSpindexerUnjamDuration = 0.3;
    public static final double kSpindexerUnjamSpeed = -0.5;
}
