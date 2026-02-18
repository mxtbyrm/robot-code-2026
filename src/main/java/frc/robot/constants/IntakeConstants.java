package frc.robot.constants;

import frc.robot.RobotConfig;

public final class IntakeConstants {
    // ==================== MOTOR CAN IDs ====================
    // 2 Kraken X60 deploy motors (left + right, mirror-inverted)
    public static final int kLeftDeployMotorId  = RobotConfig.kLeftDeployMotorId;
    public static final int kRightDeployMotorId = RobotConfig.kRightDeployMotorId;
    // 1 Kraken X60 roller motor (collects balls)
    public static final int kRollerMotorId = RobotConfig.kRollerMotorId;

    // ==================== DEPLOY CONSTANTS ====================
    // Deploy gear ratio: motor rotations per mechanism rotation
    public static final double kDeployGearRatio = RobotConfig.kDeployGearRatio;

    // Measured raw Kraken motor encoder readings (motor rotations)
    private static final double kStowedMotorRotations   = 0.0; // measured at stowed position
    private static final double kExtendedMotorRotations = RobotConfig.kIntakeExtendedMotorRotations; // measured at full deploy

    // Deploy positions in mechanism rotations (0 = stowed inside bumpers)
    // Negative = deployed outward past the bumper
    public static final double kDeployStowedRotations   = kStowedMotorRotations / kDeployGearRatio;
    public static final double kDeployExtendedRotations = kExtendedMotorRotations / kDeployGearRatio;
    public static final double kDeployHoverRotations    = kDeployExtendedRotations / 2.0;

    // Deploy PID (position control — gravity handled via lookup tables, NOT Arm_Cosine)
    // Values set in RobotConfig.java — edit there for new robot deployments.
    public static final double kDeployP = RobotConfig.kDeployP;
    public static final double kDeployI = RobotConfig.kDeployI;
    public static final double kDeployD = RobotConfig.kDeployD;
    public static final double kDeployS = RobotConfig.kDeployS; // Static friction FF (volts) — minimum voltage to overcome friction
    // TODO: tune with SysId or by slowly increasing voltage until the arm just starts to move

    // ==================== DEPLOY GRAVITY FEEDFORWARD LOOKUP ====================
    // The passive linkage (free flap) creates a non-linear torque profile:
    //   - Stowed: linkage is folded, CG is near pivot → low gravity torque
    //   - Mid-deploy: linkage starts opening → torque ramps up
    //   - Full deploy: linkage fully extended → maximum gravity torque
    // A simple cosine doesn't capture this, so we use position → voltage tables.
    // The right side carries more weight (roller/linkage mass is off-center).
    //
    // Format: { position (mechanism rotations), voltage (V) }
    // Positions are negative (0 = stowed, kDeployExtendedRotations = full deploy).
    // TODO: tune all values on the real robot using AdvantageScope current plots
    public static final double[][] kDeployLeftGravityTable = {
        { 0.0,                             RobotConfig.kDeployLeftGravityVoltages[0] },  // stowed — vertical, ~zero torque
        { kDeployExtendedRotations * 0.25, RobotConfig.kDeployLeftGravityVoltages[1] },  // 25% deploy — linkage still folded
        { kDeployExtendedRotations * 0.50, RobotConfig.kDeployLeftGravityVoltages[2] },  // 50% deploy — linkage opening
        { kDeployExtendedRotations * 0.75, RobotConfig.kDeployLeftGravityVoltages[3] },  // 75% deploy — linkage mostly open
        { kDeployExtendedRotations,        RobotConfig.kDeployLeftGravityVoltages[4] },  // full deploy — linkage extended, max torque
    };

    public static final double[][] kDeployRightGravityTable = {
        { 0.0,                             RobotConfig.kDeployRightGravityVoltages[0] },  // stowed — vertical, ~zero torque
        { kDeployExtendedRotations * 0.25, RobotConfig.kDeployRightGravityVoltages[1] },  // 25% deploy — linkage still folded, extra weight kicks in
        { kDeployExtendedRotations * 0.50, RobotConfig.kDeployRightGravityVoltages[2] },  // 50% deploy — linkage opening
        { kDeployExtendedRotations * 0.75, RobotConfig.kDeployRightGravityVoltages[3] },  // 75% deploy — linkage mostly open
        { kDeployExtendedRotations,        RobotConfig.kDeployRightGravityVoltages[4] },  // full deploy — linkage extended, max torque + extra weight
    };

    // Deploy tolerance — ~10% of full deploy travel
    // Travel = |kDeployExtendedRotations| ≈ 0.00323 mechanism rotations
    public static final double kDeployToleranceRotations = 0.0003;

    // Deploy current limit (amps) — value set in RobotConfig.java
    public static final double kDeployCurrentLimit = RobotConfig.kDeployCurrentLimit;

    // Deploy stall detection
    public static final double kDeployStallCurrentThreshold = 25.0; // Amps
    public static final int    kDeployStallCycleThreshold   = 25;   // ~0.5s

    // Deploy motion magic (smooth trapezoidal motion)
    // Values set in RobotConfig.java — edit there for new robot deployments.
    public static final double kDeployMaxVelocity     = RobotConfig.kDeployMaxVelocity;     // rotations/sec
    public static final double kDeployMaxAcceleration = RobotConfig.kDeployMaxAcceleration; // rotations/sec²

    // ==================== ROLLER CONSTANTS ====================
    // Roller speed (duty cycle) — values set in RobotConfig.java
    public static final double kRollerIntakeSpeed  = RobotConfig.kRollerIntakeSpeed;
    public static final double kRollerOuttakeSpeed = RobotConfig.kRollerOuttakeSpeed;
    public static final double kRollerCurrentLimit = RobotConfig.kRollerCurrentLimit;

    // ==================== ROLLER PHYSICAL DIMENSIONS ====================
    // Needed to compute roller surface speed for chassis speed limiting.
    // Roller surface speed MUST exceed chassis speed while intaking,
    // otherwise we push balls away instead of collecting them.
    // Values set in RobotConfig.java — edit there for new robot deployments.
    public static final double kRollerDiameterMeters = RobotConfig.kRollerDiameterMeters; // TODO: measure on robot
    public static final double kRollerGearRatio       = RobotConfig.kRollerGearRatio;      // motor rotations per roller rotation — TODO: verify

    // Kraken X60 free speed ≈ 6000 RPM = 100 RPS
    private static final double kKrakenFreeSpeedRPS = 100.0;

    // Computed roller surface speed at intake duty cycle (m/s)
    // = (motorRPS × dutyCycle / gearRatio) × (π × diameter)
    public static final double kRollerSurfaceSpeedMps =
            (kKrakenFreeSpeedRPS * kRollerIntakeSpeed / kRollerGearRatio)
            * (Math.PI * kRollerDiameterMeters);

    // ==================== INTAKE CHASSIS SPEED LIMIT ====================
    // While intaking, cap robot translation speed below roller surface speed.
    // If chassis moves faster than rollers spin, balls get pushed away.
    // Set to 85% of roller surface speed for reliable ball collection.
    public static final double kMaxChassisSpeedWhileIntaking =
            kRollerSurfaceSpeedMps * 0.85;
}
