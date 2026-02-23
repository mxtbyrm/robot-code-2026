package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.RobotConfig;

public final class ShooterConstants {
    // ==================== GAME PIECE — 2026 REBUILT "FUEL" ====================
    // 5.91-inch (15.0 cm) high-density foam ball
    public static final double kBallDiameterMeters   = Units.inchesToMeters(5.91); // 0.150m
    public static final double kBallRadiusMeters     = kBallDiameterMeters / 2.0;
    public static final double kBallMassKg           = 0.215; // ~215g (range: 203-227g)
    public static final double kBallCompressionMeters = 0.020; // ~20mm (range: 15-25mm)

    // Effective ball diameter when compressed between flywheels
    public static final double kCompressedBallDiameter = kBallDiameterMeters - kBallCompressionMeters;

    // ==================== HUB / GOAL SPECIFICATIONS (delegated to HubConstants) ====================
    public static final double kHubHeightMeters = HubConstants.kScoringOpeningHeightMeters;
    // kHubInnerDiameterMeters uses the flat-to-flat (inscribed) diameter — the minimum clearance
    // dimension. This gives conservative margin: (inscribed_radius - ball_radius).
    // Use HubConstants.kScoringOpeningCornerToCornerMeters for corner-to-corner (circumscribed).
    public static final double kHubInnerDiameterMeters = HubConstants.kScoringOpeningFlatToFlatMeters;

    // ==================== HUB FIELD POSITIONS (alliance-aware) ====================
    public static final Translation2d kBlueHubPosition = HubConstants.kBlueHubCenter;
    public static final Translation2d kRedHubPosition  = HubConstants.kRedHubCenter;

    public static Translation2d getActiveHubPosition() {
        return HubConstants.getActiveHubPosition();
    }

    // Legacy constant — use getActiveHubPosition() instead
    public static final Translation2d kHubFieldPosition = kBlueHubPosition;

    // ==================== AUTO-AIM CONSTANTS ====================
    // Minimum distance where auto-aim will engage (closer than this = too close to shoot)
    public static final double kMinShootingDistanceMeters = 1.2;
    // Maximum distance where auto-aim will engage (farther = not worth shooting)
    public static final double kMaxShootingDistanceMeters = 6.5;
    // Turret angular offset — compensate for turret not being exactly centered on robot (degrees)
    // Positive = turret is offset counterclockwise from robot forward. Set to 0 if centered.
    // Value set in RobotConfig.java — edit there for new robot deployments.
    public static final double kTurretMountOffsetDegrees = RobotConfig.kTurretMountOffsetDegrees;

    // ==================== SHOOTER GEOMETRY (measured on robot) ====================
    // Position of the shooter exit point in robot frame (X=forward, Y=left).
    // Rotated by robot heading each cycle to get the field-frame shooter position.
    // Value set in RobotConfig.java — edit there for new robot deployments.
    public static final Translation2d kShooterPositionOffset = RobotConfig.kShooterPositionOffset;

    // Height of ball exit point above the floor. Measure from carpet to the
    // point where the ball leaves the flywheel gap.
    // Value set in RobotConfig.java — edit there for new robot deployments.
    public static final double kShooterExitHeightMeters = RobotConfig.kShooterExitHeightMeters;

    // ==================== BALL-FLYWHEEL EFFICIENCY ====================
    // Ratio of actual ball exit speed to flywheel surface speed.
    // Accounts for foam ball compression, slip, friction losses.
    // v_exit = surface_speed × efficiency
    // Tune this single value on the real robot until shots consistently
    // land in the HUB at multiple distances.
    public static final double kShooterEfficiencyFactor = RobotConfig.kShooterEfficiencyFactor;

    // ==================== REAL-WORLD BIAS CORRECTION (DISTANCE-DEPENDENT) ====================
    // Multiplier applied to the final computed velocity to bridge the gap
    // between theoretical projectile physics (vacuum, no drag, no spin)
    // and real-world behavior. Compensates for:
    //   - Air drag (increases with distance — longer flight = more slowdown)
    //   - Ball spin from flywheel contact (creates lift or draw)
    //   - Foam ball deformation and energy loss in flight
    //   - Environmental factors (humidity, worn balls, altitude)
    //
    // The bias is linearly interpolated between near and far values:
    //   bias(d) = nearBias + (farBias − nearBias) × (d − minDist) / (maxDist − minDist)
    //
    // Close shots (2m): short flight time → minimal drag → bias ≈ 1.0
    // Far shots  (7m): long  flight time → significant drag → bias > 1.0
    //
    // 1.00 = pure theory (no correction)
    // >1.0 = real world needs MORE power than theory (typical: drag losses)
    // <1.0 = real world needs LESS power than theory (unlikely)
    //
    // Tuning:
    //   1. Start both at 1.00 (pure physics)
    //   2. Shoot at kMinShootingDistanceMeters — if short, increase Near
    //   3. Shoot at kMaxShootingDistanceMeters — if short, increase Far
    //   4. Verify mid-range shots land without re-tuning (interpolation handles it)
    public static final double kBiasFactorNear = RobotConfig.kBiasFactorNear; // at kMinShootingDistanceMeters
    public static final double kBiasFactorFar  = RobotConfig.kBiasFactorFar;  // at kMaxShootingDistanceMeters

    // ==================== ARC / BASKETBALL CONSTRAINTS ====================
    // Minimum height (meters) the ball must peak ABOVE the Hub opening.
    // Like basketball: ball must arc up and come DOWN into the basket.
    // Higher = safer arc (swish), lower = flatter trajectory (more bounce-out risk).
    // 0.3m (~12 inches) gives a clean descending arc at all distances.
    public static final double kMinApexClearanceMeters = 0.30;

    // ==================== TABLE GENERATION ====================
    // Step size for physics-computed lookup table entries.
    // Smaller = more entries = smoother interpolation, but diminishing returns.
    public static final double kTableDistanceStepMeters = 0.5;

    // Whether shoot-while-moving compensation is enabled.
    // Set to false during initial tuning, enable once the static aim is dialed in.
    public static final boolean kShootWhileMovingEnabled = true;

    // ==================== ALLIANCE ZONE DUMP ====================
    // Fixed shooter parameters for dumping FUEL from neutral zone into alliance zone.
    // No auto-aim — turret faces backward, hood and flywheel set to a fixed lob shot.
    // Values set in RobotConfig.java — must be re-tuned on each physical robot.
    public static final double kAllianceZoneFlywheelRPS    = RobotConfig.kAllianceZoneFlywheelRPS;
    public static final double kAllianceZoneHoodAngleDeg   = RobotConfig.kAllianceZoneHoodAngleDeg;
    public static final double kAllianceZoneTurretAngleDeg = RobotConfig.kAllianceZoneTurretAngleDeg;
    public static final double kAllianceZoneFeederSpeed    = RobotConfig.kAllianceZoneFeederSpeed;

    // ==================== TURRET WRAPAROUND ====================
    // When the turret is within this many degrees of either limit, the drivetrain
    // should auto-rotate to bring the target back toward turret center.
    // This prevents the turret from hitting hard limits during fast rotations.
    // Uses hysteresis: activates at kTurretWraparoundThresholdDeg,
    // deactivates at kTurretWraparoundReleaseThresholdDeg.
    public static final double kTurretWraparoundThresholdDeg        = 150.0;
    public static final double kTurretWraparoundReleaseThresholdDeg = 140.0;
    // Max additional rotation speed (rad/s) the wraparound hint can add.
    // Scales proportionally from 0 at the release threshold to max at the hard limit.
    public static final double kTurretWraparoundMaxHintRadPerSec = 1.5;

    // ==================== PRE-FEED REVERSE ====================
    // Before feeding into the shooter, briefly reverse feeder + spindexer
    // at low speed to create ball clearance and prevent jams.
    // Like pulling the bolt back before firing — creates space so balls
    // feed cleanly one at a time instead of jamming against each other.
    // Values set in RobotConfig.java — edit there for new robot deployments.
    public static final double kPreFeedReverseDurationSeconds  = RobotConfig.kPreFeedReverseDurationSeconds;
    public static final double kPreFeedReverseFeederSpeed      = RobotConfig.kPreFeedReverseFeederSpeed;
    public static final double kPreFeedReverseSpindexerSpeed   = RobotConfig.kPreFeedReverseSpindexerSpeed;

    // ==================== ENDGAME ====================
    // Time remaining in Teleop (seconds) to trigger "endgame dump" mode.
    // In the last seconds, dump all remaining FUEL regardless of Hub state.
    public static final double kEndgameDumpTimeSeconds = 10.0;

    // ==================== MOTOR CAN IDs ====================
    // Values set in RobotConfig.java — edit there for new robot deployments.
    public static final int kFlywheelMotorId = RobotConfig.kFlywheelMotorId; // Kraken — drives both top and bottom flywheels
    public static final int kHoodMotorId     = RobotConfig.kHoodMotorId;     // Kraken — adjusts hood angle
    public static final int kTurretMotorId   = RobotConfig.kTurretMotorId;   // Kraken — rotates turret for aiming

    // ==================== FLYWHEEL PHYSICAL CONSTANTS ====================
    // Bottom flywheel: 4-inch diameter wheels + 2 iron inertia wheels (0.7kg each)
    public static final double kBottomFlywheelDiameterMeters      = RobotConfig.kBottomFlywheelDiameterMeters; // 0.1016m
    public static final double kBottomFlywheelRadiusMeters        = kBottomFlywheelDiameterMeters / 2.0;
    public static final double kBottomFlywheelCircumferenceMeters = kBottomFlywheelDiameterMeters * Math.PI;
    public static final double kIronWheelMassKg  = 0.7;  // each iron inertia wheel
    public static final int    kIronWheelCount   = 2;

    // Top (hood) flywheel: 2-inch diameter wheels, 2 rows
    public static final double kTopFlywheelDiameterMeters      = RobotConfig.kTopFlywheelDiameterMeters; // 0.0508m
    public static final double kTopFlywheelRadiusMeters        = kTopFlywheelDiameterMeters / 2.0;
    public static final double kTopFlywheelCircumferenceMeters = kTopFlywheelDiameterMeters * Math.PI;

    // Surface speed equalization gear ratio:
    // Both flywheels are driven by ONE motor, geared so surface speed is equal.
    // Surface speed = RPM * circumference
    // For equal surface speed: RPM_top * C_top = RPM_bottom * C_bottom
    // RPM_top / RPM_bottom = C_bottom / C_top = 4" / 2" = 2.0
    // So the top flywheel spins 2x faster than the bottom flywheel.
    // This means the gear ratio from motor to bottom is X, and motor to top is 2X.
    // Motor to bottom flywheel: 2:1 (2 motor rotations per 1 flywheel rotation)
    public static final double kFlywheelGearRatio    = RobotConfig.kFlywheelGearRatio;
    // Bottom to top flywheel: 1:2 (top spins 2x faster for equal surface speed)
    public static final double kTopToBottomGearRatio = RobotConfig.kTopToBottomGearRatio;

    // ==================== FLYWHEEL PID / FF ====================
    // Velocity PID for flywheel speed control
    // Values set in RobotConfig.java — edit there for new robot deployments.
    public static final double kFlywheelP = RobotConfig.kFlywheelP;
    public static final double kFlywheelI = RobotConfig.kFlywheelI;
    public static final double kFlywheelD = RobotConfig.kFlywheelD;
    public static final double kFlywheelS = RobotConfig.kFlywheelS;  // Static friction
    public static final double kFlywheelV = RobotConfig.kFlywheelV;  // Velocity FF (volts per rps)
    public static final double kFlywheelA = RobotConfig.kFlywheelA;  // Acceleration FF

    // Acceptable velocity error before we consider flywheel "at speed"
    public static final double kFlywheelToleranceRPS = RobotConfig.kFlywheelToleranceRPS; // rotations per second

    // Idle pre-spin speed when out of shooting range (keeps flywheel warm for faster spin-up)
    public static final double kIdleFlywheelRPS = RobotConfig.kIdleFlywheelRPS;

    // Maximum flywheel RPS used to normalize shooter speed for feeder/spindexer scaling.
    // Feeder and spindexer duty cycles are computed as:
    //   duty = ratio × (targetFlywheelRPS / kMaxFlywheelRPS)
    // This value should be >= the highest RPS in the shooting table.
    // TODO: set from actual shooting table max after physics tuning
    public static final double kMaxFlywheelRPS = RobotConfig.kMaxFlywheelRPS;

    // ==================== HOOD CONSTANTS ====================
    // Hood gear ratio: 2:1 (2 motor rotations per mechanism rotation)
    public static final double kHoodGearRatio = RobotConfig.kHoodGearRatio;

    // Hood angle range (measured from horizontal / parallel to ground)
    public static final double kHoodMinAngleDegrees = 27.5;
    public static final double kHoodMaxAngleDegrees = 42.5;

    // Values set in RobotConfig.java — edit there for new robot deployments.
    private static final double kHoodMinMotorRotations = RobotConfig.kHoodMinMotorRotations;
    private static final double kHoodMaxMotorRotations = RobotConfig.kHoodMaxMotorRotations;

    // Hood positions in mechanism rotations (for motor controller)
    public static final double kHoodMinRotations = kHoodMinMotorRotations / kHoodGearRatio;
    public static final double kHoodMaxRotations = kHoodMaxMotorRotations / kHoodGearRatio;

    // ==================== EXIT ANGLE MAPPING ====================
    // The hood mechanical angle maps to a ball exit angle.
    // Ball exits roughly perpendicular to the hood surface, so:
    //   exit_angle_from_horizontal = 90° − hood_angle
    // When hood is at minimum (open): ball exits steep (high angle).
    // When hood is at maximum (closed): ball exits flat (low angle).
    public static final double kExitAngleAtMinHood = 90.0 - kHoodMinAngleDegrees;
    public static final double kExitAngleAtMaxHood = 90.0 - kHoodMaxAngleDegrees;

    // Hood PID (position control)
    // Values set in RobotConfig.java — edit there for new robot deployments.
    public static final double kHoodP = RobotConfig.kHoodP;
    public static final double kHoodI = RobotConfig.kHoodI;
    public static final double kHoodD = RobotConfig.kHoodD;
    public static final double kHoodG = RobotConfig.kHoodG; // Gravity feedforward (volts)

    // Hood tolerance
    public static final double kHoodToleranceDegrees  = 0.5;
    public static final double kHoodToleranceRotations = kHoodToleranceDegrees / 360.0;

    // ==================== TURRET CONSTANTS ====================
    // Turret gear ratio: motor rotations per mechanism rotation
    // Value set in RobotConfig.java — edit there for new robot deployments.
    public static final double kTurretGearRatio = RobotConfig.kTurretGearRatio;

    // Turret rotation limits (degrees from center / forward)
    // Narrowed from ±180° to ±175° to prevent oscillation at the wrap boundary.
    public static final double kTurretMinAngleDegrees = -175.0;
    public static final double kTurretMaxAngleDegrees =  175.0;

    // Values set in RobotConfig.java — edit there for new robot deployments.
    private static final double kTurretCCWLimitMotorRotations = RobotConfig.kTurretCCWLimitMotorRotations;
    private static final double kTurretCWLimitMotorRotations  = RobotConfig.kTurretCWLimitMotorRotations;

    // Turret positions in mechanism rotations (for motor controller)
    // Derived from the two hard stop measurements and gear ratio
    public static final double kTurretMinRotations =
            kTurretCCWLimitMotorRotations / kTurretGearRatio;
    public static final double kTurretMaxRotations =
            kTurretCWLimitMotorRotations / kTurretGearRatio;

    // Turret PID (position control)
    // Values set in RobotConfig.java — edit there for new robot deployments.
    public static final double kTurretP = RobotConfig.kTurretP;
    public static final double kTurretI = RobotConfig.kTurretI;
    public static final double kTurretD = RobotConfig.kTurretD;
    public static final double kTurretSpringFeedForwardVPerDeg = RobotConfig.kTurretSpringFeedForwardVPerDeg;

    // Turret tolerance
    public static final double kTurretToleranceDegrees  = 1.0;
    public static final double kTurretToleranceRotations = kTurretToleranceDegrees / 360.0;

    // ==================== TURRET / HOOD PHYSICAL DIMENSIONS ====================
    /** Height of turret rotation axis above ground (meters). ~18 inches. */
    public static final double kTurretHeightMeters = RobotConfig.kTurretHeightMeters;
    /** Height of hood pivot above ground (meters). Slightly above turret. */
    public static final double kHoodHeightMeters = RobotConfig.kHoodHeightMeters;

    // ==================== FLYWHEEL RECOVERY ====================
    /** Cooldown time after a ball exits to let flywheel recover speed (seconds). */
    public static final double kFlywheelRecoverySeconds = 0.15;

    // ==================== RUNTIME HEALTH ====================
    /** Consecutive fault cycles before marking unhealthy (~1s at 50Hz). */
    public static final int kFaultCycleThreshold = 50;

    // ==================== SHOOTING LOOKUP TABLE ====================
    // Generated at robot init by ShooterPhysics from projectile motion equations.
    // No hardcoded values — everything derived from physical constants above.
    // See ShooterPhysics.java for the computation.
}
