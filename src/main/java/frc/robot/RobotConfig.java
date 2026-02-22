package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Single source of truth for all per-robot hardware measurements and tunable values.
 *
 * <p>When deploying to a new robot, only edit THIS file.
 * All other files (constants, subsystems) read from here — no other files
 * need to change for a new robot deployment.
 *
 * <p>Sections:
 * <ol>
 *   <li>Operator — controller ports and deadband
 *   <li>LED Hardware — PWM port and LED count
 *   <li>Swerve CAN Wiring — Pigeon2 and module motor/encoder IDs
 *   <li>Swerve Module Hardware — gear ratios and wheel diameter
 *   <li>Swerve PID / FF / Driver Params — all swerve tuning values
 *   <li>Shooter CAN Wiring — flywheel, hood, turret motor IDs
 *   <li>Shooter Hardware — flywheel/hood/turret physical dimensions
 *   <li>Shooter Geometry — position offset and exit height
 *   <li>Hood Calibration — motor encoder hard-stop readings
 *   <li>Turret Calibration — gear ratio, hard-stop readings, mount offset
 *   <li>Shooter Tuning — efficiency, bias, PID, FF values
 *   <li>Feeder & Spindexer CAN + Tuning — motor IDs and speed ratios
 *   <li>Intake CAN Wiring — deploy and roller motor IDs
 *   <li>Intake Hardware / Calibration — gear ratios, travel, dimensions, PID, gravity voltages
 *   <li>Camera Hardware — mounting heights, pitch, and offsets
 *   <li>Vision Tuning — standard deviations and ball-chase PID
 *   <li>Ball Detection Tuning — current thresholds and baselines
 *   <li>Swerve Geometry — track width and wheelbase
 *   <li>CANcoder Offsets — absolute encoder zero-offset per module
 * </ol>
 */
public final class RobotConfig {

    private RobotConfig() {} // utility class

    // ==================== 1. OPERATOR ====================

    /** USB port index for the driver's Xbox controller. */
    public static final int kDriverControllerPort = 0;

    /** USB port index for the operator's Xbox controller. */
    public static final int kOperatorControllerPort = 1;

    /** Joystick axis deadband (applies to both driver and operator). */
    public static final double kDeadband = 0.1;

    // ==================== 2. LED HARDWARE ====================

    /** PWM port for the addressable LED strip. */
    public static final int kLedPort = 0;

    /** Number of LEDs on the strip. */
    public static final int kLedCount = 60;

    // ==================== 3. SWERVE CAN WIRING ====================

    /** CAN ID for the Pigeon2 IMU on the CANivore bus. */
    public static final int kPigeonId = 13;

    // Front Left Module
    public static final int kFrontLeftDriveMotorId = 1;
    public static final int kFrontLeftSteerMotorId = 2;
    public static final int kFrontLeftEncoderId    = 3;

    // Front Right Module
    public static final int kFrontRightDriveMotorId = 4;
    public static final int kFrontRightSteerMotorId = 5;
    public static final int kFrontRightEncoderId    = 6;

    // Back Left Module
    public static final int kBackLeftDriveMotorId = 7;
    public static final int kBackLeftSteerMotorId = 8;
    public static final int kBackLeftEncoderId    = 9;

    // Back Right Module
    public static final int kBackRightDriveMotorId = 10;
    public static final int kBackRightSteerMotorId = 11;
    public static final int kBackRightEncoderId    = 12;

    // ==================== 4. SWERVE MODULE HARDWARE ====================

    /** Drive gear ratio: motor rotations per wheel rotation. MK4i L2 = 6.75:1. */
    public static final double kDriveGearRatio = 6.75;

    /** Steer gear ratio: motor rotations per module azimuth rotation. ~21.43:1 for all MK4i. */
    public static final double kSteerGearRatio = 150.0 / 7.0;

    /** Wheel diameter in meters (4-inch nominal). */
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);

    // ==================== 5. SWERVE PID / FF / DRIVER PARAMS ====================

    // Drive motor PID + FF (velocity control)
    // ── Unit basis: SensorToMechanismRatio = 6.75, so all feedback is in WHEEL-SHAFT rotations.
    //    Setpoints are passed as wheel-shaft RPS; gains must match that unit.
    //
    //    kV derivation:
    //      Motor free speed (Kraken X60 FOC @ 12 V) ≈ 5800 RPM = 96.67 RPS (rotor)
    //      Wheel-shaft free speed = 96.67 / 6.75 = 14.32 RPS
    //      kV = 12 V / 14.32 RPS = 0.84 V per (wheel-shaft RPS)
    //
    //    Run SysId drive routine to refine kS, kV, kA on real carpet.
    public static final double kDriveP = 0.5;
    public static final double kDriveI = 0.0;
    public static final double kDriveD = 0.0;
    public static final double kDriveS = 0.15;  // Static friction (V) — Kraken X60 on carpet
    public static final double kDriveV = 0.84;  // Velocity FF (V / wheel-shaft-RPS); see derivation above
    public static final double kDriveA = 0.01;  // Acceleration FF (V / (wheel-shaft-RPS/s)); refine with SysId

    // Steer motor PID + static FF (position control, mechanism = azimuth rotations 0–1)
    // kS compensates for stiction in the 150/7 azimuth gearbox so the module snaps
    // to angle quickly even from small errors. Run SysId steer routine to refine.
    public static final double kSteerP = 50.0;
    public static final double kSteerI = 0.0;
    public static final double kSteerD = 0.5;
    public static final double kSteerS = 0.12;  // Static friction (V) — steer gearbox stiction

    // Auto trajectory following PID (Choreo/PathPlanner)
    public static final double kAutoTranslationP = 10.0;
    public static final double kAutoTranslationI = 0.0;
    public static final double kAutoTranslationD = 0.0;
    public static final double kAutoRotationP    = 7.5;
    public static final double kAutoRotationI    = 0.0;
    public static final double kAutoRotationD    = 0.0;

    // Heading lock PID (ProfiledPIDController)
    public static final double kHeadingLockP = 5.0;
    public static final double kHeadingLockI = 0.0;
    public static final double kHeadingLockD = 0.3;

    /** Max angular acceleration for heading-lock trapezoidal profile (rad/s²). */
    public static final double kMaxAngularAccelRadiansPerSecondSq = 8.0;

    /** Speed multiplier when slow-mode trigger is held. */
    public static final double kSlowModeSpeedMultiplier = 0.35;

    /** Rotation multiplier when slow-mode trigger is held. */
    public static final double kSlowModeRotMultiplier = 0.40;

    /** Translation slew rate limiter (m/s per second). */
    public static final double kTranslationSlewRate = 6.0;

    /** Rotation slew rate limiter (rad/s per second). */
    public static final double kRotationSlewRate = 8.0;

    // Swerve motor current limits
    /** Drive motor supply current limit (amps). Lower = less heat but less torque. */
    public static final double kDriveCurrentLimit = 60.0;
    /** Drive motor stator current limit (amps). Prevents wheel slip under hard acceleration. */
    public static final double kDriveStatorCurrentLimit = 80.0;
    /** Steer motor supply current limit (amps). Low mass azimuth — 30A is plenty. */
    public static final double kSteerCurrentLimit = 30.0;

    // ==================== 6. SHOOTER CAN WIRING ====================

    /** CAN ID for the flywheel Kraken motor. */
    public static final int kFlywheelMotorId = 20;

    /** CAN ID for the hood Kraken motor. */
    public static final int kHoodMotorId = 21;

    /** CAN ID for the turret Kraken motor. */
    public static final int kTurretMotorId = 22;

    // ==================== 7. SHOOTER HARDWARE ====================

    /**
     * Motor rotations per bottom flywheel rotation.
     * Both flywheels are driven by ONE motor. 2:1 means 2 motor rotations per flywheel rotation.
     */
    public static final double kFlywheelGearRatio = 2.0;

    /**
     * Bottom-to-top flywheel gear ratio: top spins this many times per bottom rotation.
     * Sized so top and bottom surface speeds are equal.
     * 4-inch bottom, 2-inch top → top must spin 2× faster.
     */
    public static final double kTopToBottomGearRatio = 2.0;

    /** Bottom flywheel diameter (meters). 4-inch nominal. */
    public static final double kBottomFlywheelDiameterMeters = Units.inchesToMeters(4.0);

    /** Top (hood) flywheel diameter (meters). 2-inch nominal. */
    public static final double kTopFlywheelDiameterMeters = Units.inchesToMeters(2.0);

    /** Hood gear ratio: motor rotations per hood mechanism rotation. */
    public static final double kHoodGearRatio = 2.0;

    /** Height of turret rotation axis above the floor (meters). ~18 inches. */
    public static final double kTurretHeightMeters = 0.45;

    /** Height of hood pivot above the floor (meters). Slightly above turret. */
    public static final double kHoodHeightMeters = 0.50;

    // ==================== 8. SHOOTER GEOMETRY ====================

    /**
     * Position of the shooter exit point relative to the robot's odometry center,
     * expressed in the robot frame (X = forward, Y = left), in meters.
     *
     * <p>How to measure:
     * <ol>
     *   <li>Place the robot on a flat surface. Mark the robot odometry center on the floor
     *       (center of the four swerve module wheel contact patches).
     *   <li>Measure the forward (X) and lateral (Y) distance from that center point to the
     *       ball exit gap between the flywheels.
     *   <li>Forward of center = positive X. Left of center = positive Y.
     * </ol>
     *
     * <p>Example: if the shooter is 10 cm forward and 2 cm to the right of robot center,
     * use {@code new Translation2d(0.10, -0.02)}.
     *
     * <p>Set to {@code Translation2d(0, 0)} if the shooter is at the robot center
     * (or for initial bring-up before measurement).
     *
     * <p>TODO: measure on real robot
     */
    public static final Translation2d kShooterPositionOffset = new Translation2d(0.0, 0.0);

    /**
     * Height of the ball exit point above the floor, in meters.
     * Measure from carpet to the point where the ball leaves the flywheel gap.
     *
     * <p>How to measure:
     * <ol>
     *   <li>Hold a ruler vertically next to the shooter.
     *   <li>Read the height at the center of the flywheel gap.
     * </ol>
     *
     * <p>Example: 24 inches ≈ 0.610 m.
     *
     * <p>TODO: measure on real robot
     */
    public static final double kShooterExitHeightMeters = Units.inchesToMeters(24.0);

    // ==================== 9. HOOD CALIBRATION ====================

    /**
     * Kraken motor encoder reading (motor rotations) when the hood is at its
     * minimum (most open) position — ball exits at the steepest arc.
     *
     * <p>How to calibrate:
     * <ol>
     *   <li>Power on the robot with the hood physically at its minimum hard stop.
     *   <li>In Phoenix Tuner X, read the TalonFX position signal for the hood motor.
     *   <li>Record that value here (units: motor rotations).
     * </ol>
     *
     * <p>TODO: calibrate on real robot
     */
    public static final double kHoodMinMotorRotations = 0.0; // placeholder

    /**
     * Kraken motor encoder reading (motor rotations) when the hood is at its
     * maximum (most closed) position — ball exits at the flattest trajectory.
     *
     * <p>How to calibrate:
     * <ol>
     *   <li>Power on. Manually drive the hood motor to the maximum hard stop.
     *   <li>In Phoenix Tuner X, read the TalonFX position signal for the hood motor.
     *   <li>Record that value here (units: motor rotations).
     * </ol>
     *
     * <p>TODO: calibrate on real robot
     */
    public static final double kHoodMaxMotorRotations = 0.0; // placeholder

    // ==================== 10. TURRET CALIBRATION ====================

    /**
     * Gear ratio from Kraken motor shaft to turret output shaft.
     * Units: motor rotations per one full turret rotation.
     *
     * <p>How to measure:
     * <ol>
     *   <li>Mark a reference line on the turret and the fixed frame.
     *   <li>Using Phoenix Tuner X, zero the motor encoder.
     *   <li>Manually (or via Tuner) rotate the turret exactly one full revolution.
     *   <li>Read the motor encoder value — that is the gear ratio.
     * </ol>
     *
     * <p>TODO: measure on real robot (placeholder is 100)
     */
    public static final double kTurretGearRatio = 100.0; // placeholder

    /**
     * Kraken motor encoder reading (motor rotations) at the turret's CCW (positive angle)
     * hard stop.
     *
     * <p>How to calibrate:
     * <ol>
     *   <li>Power on with turret centered (encoder zeroed at 0).
     *   <li>Slowly rotate turret CCW until it hits the hard stop.
     *   <li>In Phoenix Tuner X, read the TalonFX position — record here.
     * </ol>
     *
     * <p>TODO: calibrate on real robot
     */
    public static final double kTurretCCWLimitMotorRotations = 0.0; // placeholder

    /**
     * Kraken motor encoder reading (motor rotations) at the turret's CW (negative angle)
     * hard stop.
     *
     * <p>How to calibrate:
     * <ol>
     *   <li>Power on with turret centered (encoder zeroed at 0).
     *   <li>Slowly rotate turret CW until it hits the hard stop.
     *   <li>In Phoenix Tuner X, read the TalonFX position (negative value) — record here.
     * </ol>
     *
     * <p>TODO: calibrate on real robot
     */
    public static final double kTurretCWLimitMotorRotations = 0.0; // placeholder

    /**
     * Angular offset (degrees) between the turret's mechanical "zero" direction and the
     * robot's forward direction. Positive = turret zero is CCW from robot forward.
     *
     * <p>How to measure:
     * <ol>
     *   <li>Zero the turret encoder with turret pointing straight forward.
     *   <li>If the turret is not perfectly aligned with the robot forward axis, measure
     *       the angle difference with a digital angle gauge.
     *   <li>Set this value so that turret angle 0° commands the turret to face exactly
     *       forward on the robot.
     * </ol>
     *
     * <p>Example: if the turret mount is 2° CCW from robot forward, use {@code 2.0}.
     * Set to {@code 0.0} if the turret is centered (or before measurement).
     */
    public static final double kTurretMountOffsetDegrees = 0.0;

    // ==================== 11. SHOOTER TUNING ====================

    /**
     * Ratio of actual ball exit speed to flywheel surface speed.
     * Accounts for foam ball compression, slip, and friction losses.
     * v_exit = surface_speed × efficiency.
     * Tune on the real robot until shots consistently land in the HUB.
     */
    public static final double kShooterEfficiencyFactor = 0.30;

    /**
     * Velocity bias multiplier at the minimum shooting distance.
     * 1.00 = pure physics. Increase if real shots fall short at close range.
     */
    public static final double kBiasFactorNear = 1.00;

    /**
     * Velocity bias multiplier at the maximum shooting distance.
     * 1.00 = pure physics. Increase if real shots fall short at long range (drag).
     */
    public static final double kBiasFactorFar = 1.00;

    // Flywheel velocity PID + FF
    // ── Unit basis: SensorToMechanismRatio NOT set (assumed 1.0), so feedback is in ROTOR RPS.
    //    kMaxFlywheelRPS = 90 > mechanism free speed (48.33 RPS), confirming rotor units.
    //
    //    kV derivation (rotor units):
    //      kV = 12 V / 96.67 RPS_rotor = 0.124 ≈ 0.12 V per rotor-RPS  ← already correct
    //
    //    Run SysId flywheel routine to refine kS, kA on real robot.
    public static final double kFlywheelP = 0.5;
    public static final double kFlywheelI = 0.0;
    public static final double kFlywheelD = 0.0;
    public static final double kFlywheelS = 0.10;  // Static friction (V) — estimated Kraken X60 stiction
    public static final double kFlywheelV = 0.12;  // Velocity FF (V / rotor-RPS) — physics-verified
    public static final double kFlywheelA = 0.01;  // Acceleration FF (V / (rotor-RPS/s)) — refine with SysId

    /** Acceptable velocity error before flywheel is considered "at speed" (rps). */
    public static final double kFlywheelToleranceRPS = 2.0;

    /** Idle pre-spin speed to reduce spin-up time when a shot is needed (rps). */
    public static final double kIdleFlywheelRPS = 15.0;

    /**
     * Maximum flywheel RPS — used to normalize feeder/spindexer duty cycles.
     * Should be >= the highest RPS in the shooting table.
     */
    public static final double kMaxFlywheelRPS = 90.0;

    // Hood PID + gravity FF
    public static final double kHoodP = 8.0;
    public static final double kHoodI = 0.0;
    public static final double kHoodD = 0.2;
    public static final double kHoodG = 0.15; // Gravity feedforward (volts)

    // Turret position PID
    public static final double kTurretP = 30.0;
    public static final double kTurretI = 0.0;
    public static final double kTurretD = 0.5;
    // Spring cable feedforward: compensates for Vulcan spring restoring torque.
    // The spring pulls the turret back toward 0° proportional to angle.
    // Increase this value until the turret holds position without drifting back.
    // Units: volts per degree of turret rotation from center.
    // Start at 0.0, tune by increasing ~0.005 at a time on the robot.
    public static final double kTurretSpringFeedForwardVPerDeg = 0.0;

    // Alliance zone dump — fixed params for lobbing FUEL into the alliance zone.
    // No auto-aim: turret faces backward, hood and flywheel set to a fixed lob shot.
    // Must be re-tuned on each physical robot to land balls in the alliance zone.
    public static final double kAllianceZoneFlywheelRPS      = 35.0;  // low-power lob
    public static final double kAllianceZoneHoodAngleDeg     = 40.0;  // steep arc
    public static final double kAllianceZoneTurretAngleDeg   = 172.0; // face backward (buffered from ±175° limit)
    public static final double kAllianceZoneFeederSpeed      = 0.60;  // moderate feed

    // Pre-feed reverse — brief reverse pulse before feeding to create ball clearance.
    // Tune: if balls jam entering the flywheel, increase duration or slow the speeds.
    public static final double kPreFeedReverseDurationSeconds = 0.12;  // 120 ms pulse
    public static final double kPreFeedReverseFeederSpeed     = -0.25; // slow reverse
    public static final double kPreFeedReverseSpindexerSpeed  = -0.15; // slow reverse

    // ==================== 12. FEEDER & SPINDEXER CAN + TUNING ====================

    /** CAN ID for the feeder Kraken motor. */
    public static final int kFeederMotorId = 23;

    /**
     * DIO port for the beam-break sensor at the feeder exit.
     * TODO: set to actual DIO port on real robot.
     */
    public static final int kBeamBreakDIOPort = 0;

    /** CAN ID for the spindexer Kraken motor. */
    public static final int kSpindexerMotorId = 27;

    /**
     * Feeder duty-cycle ratio relative to normalized flywheel RPS.
     * feeder_duty = kFeederSpeedRatio × (targetFlywheelRPS / kMaxFlywheelRPS)
     */
    public static final double kFeederSpeedRatio = 0.75;

    /** Spindexer duty-cycle during ball intake (slow continuous spin). */
    public static final double kSpindexerIntakeSpeed = 0.4;

    /**
     * Spindexer-to-feeder speed ratio while shooting.
     * spindexer_duty = kSpindexerToFeederRatio × feeder_duty (always < feeder by design).
     */
    public static final double kSpindexerToFeederRatio = 0.65;

    /** Feeder motor supply current limit (amps). */
    public static final double kFeederCurrentLimit = 30.0;

    /** Spindexer motor supply current limit (amps). */
    public static final double kSpindexerCurrentLimit = 30.0;

    // ==================== 13. INTAKE CAN WIRING ====================

    /** CAN ID for the left deploy Kraken motor. */
    public static final int kLeftDeployMotorId = 24;

    /** CAN ID for the right deploy Kraken motor (mirror-inverted). */
    public static final int kRightDeployMotorId = 25;

    /** CAN ID for the roller Kraken motor. */
    public static final int kRollerMotorId = 26;

    // ==================== 14. INTAKE HARDWARE / CALIBRATION ====================

    /** Motor rotations per deploy mechanism rotation. */
    public static final double kDeployGearRatio = 4.0;

    /**
     * Raw Kraken motor encoder reading (motor rotations) at full deploy.
     * Measured at the extended hard stop.
     * TODO: re-measure after any intake rebuild.
     */
    public static final double kIntakeExtendedMotorRotations = -0.01293;

    /** Roller wheel diameter (meters). TODO: measure on robot. */
    public static final double kRollerDiameterMeters = Units.inchesToMeters(2.0);

    /** Motor rotations per roller rotation. TODO: verify against CAD. */
    public static final double kRollerGearRatio = 5.0;

    // Deploy position PID + static FF
    public static final double kDeployP = 40.0;
    public static final double kDeployI = 0.0;
    public static final double kDeployD = 1.0;
    public static final double kDeployS = 0.12; // Static friction (volts)

    // Deploy Motion Magic trapezoidal profile
    public static final double kDeployMaxVelocity     = 4.0; // rotations/sec
    public static final double kDeployMaxAcceleration = 8.0; // rotations/sec²

    /** Deploy motor supply current limit (amps). Lower because it's just swinging an arm. */
    public static final double kDeployCurrentLimit = 30.0;

    /** Roller motor supply current limit (amps). Higher to handle ball contact loads. */
    public static final double kRollerCurrentLimit = 40.0;

    /** Roller duty cycle while actively collecting balls inward. */
    public static final double kRollerIntakeSpeed = 0.8;

    /** Roller duty cycle while ejecting balls outward (negative = reverse). */
    public static final double kRollerOuttakeSpeed = -0.6;

    /**
     * Gravity FF voltage lookup for the LEFT deploy motor at 5 evenly-spaced positions.
     * Index 0 = stowed (0% deploy), index 4 = full deploy (100%).
     * The Y values correspond to kDeployExtendedRotations × {0.0, 0.25, 0.50, 0.75, 1.0}.
     * TODO: tune on real robot using AdvantageScope current plots.
     */
    public static final double[] kDeployLeftGravityVoltages = { 0.0, 0.10, 0.25, 0.35, 0.40 };

    /**
     * Gravity FF voltage lookup for the RIGHT deploy motor at 5 evenly-spaced positions.
     * Right side carries extra weight (roller + linkage mass off-center).
     * TODO: tune on real robot using AdvantageScope current plots.
     */
    public static final double[] kDeployRightGravityVoltages = { 0.0, 0.15, 0.35, 0.50, 0.60 };

    // ==================== 15. CAMERA HARDWARE ====================
    //
    // Each camera has its own 6-DOF pose relative to the robot center:
    //   X = forward (meters), Y = left (meters), Z = up (meters)
    //   Roll = rotation about X axis (radians)
    //   Pitch = rotation about Y axis (radians, negative = tilted upward in WPILib convention)
    //   Yaw = rotation about Z axis (radians, CCW positive from robot forward)
    //
    // How to measure:
    //   1. Place robot on flat surface. Mark robot center (center of 4 wheel contact patches).
    //   2. Measure X, Y, Z from robot center to camera optical center.
    //   3. Use a digital angle gauge to measure roll, pitch, yaw of the camera body.
    //   4. All values are TODO — fill in after physical measurement on each robot.

    // ---- Front-Left AprilTag Camera ----
    // Mounted at the front-left corner of the robot, facing 45° outward (bisects the corner).
    public static final double kFrontLeftCamX     = Units.inchesToMeters(11.375);  // TODO: measure
    public static final double kFrontLeftCamY     = Units.inchesToMeters(11.375);  // TODO: measure
    public static final double kFrontLeftCamZ     = Units.inchesToMeters(12.0);    // TODO: measure
    public static final double kFrontLeftCamRoll  = 0.0;                           // TODO: measure
    public static final double kFrontLeftCamPitch = Units.degreesToRadians(-15.0); // TODO: measure (negative = upward tilt)
    public static final double kFrontLeftCamYaw   = Units.degreesToRadians(45.0);  // TODO: measure

    // ---- Front-Right AprilTag Camera ----
    // Mounted at the front-right corner of the robot, facing 45° outward.
    public static final double kFrontRightCamX     = Units.inchesToMeters(11.375);  // TODO: measure
    public static final double kFrontRightCamY     = -Units.inchesToMeters(11.375); // TODO: measure
    public static final double kFrontRightCamZ     = Units.inchesToMeters(12.0);    // TODO: measure
    public static final double kFrontRightCamRoll  = 0.0;                           // TODO: measure
    public static final double kFrontRightCamPitch = Units.degreesToRadians(-15.0); // TODO: measure
    public static final double kFrontRightCamYaw   = Units.degreesToRadians(-45.0); // TODO: measure

    // ---- Back-Left AprilTag Camera ----
    // Mounted at the back-left corner of the robot, facing 135° from forward (45° from back).
    public static final double kBackLeftCamX     = -Units.inchesToMeters(11.375); // TODO: measure
    public static final double kBackLeftCamY     = Units.inchesToMeters(11.375);  // TODO: measure
    public static final double kBackLeftCamZ     = Units.inchesToMeters(12.0);    // TODO: measure
    public static final double kBackLeftCamRoll  = 0.0;                           // TODO: measure
    public static final double kBackLeftCamPitch = Units.degreesToRadians(-15.0); // TODO: measure
    public static final double kBackLeftCamYaw   = Units.degreesToRadians(135.0); // TODO: measure

    // ---- Back-Right AprilTag Camera ----
    // Mounted at the back-right corner of the robot, facing 135° right of forward (45° from back).
    public static final double kBackRightCamX     = -Units.inchesToMeters(11.375); // TODO: measure
    public static final double kBackRightCamY     = -Units.inchesToMeters(11.375); // TODO: measure
    public static final double kBackRightCamZ     = Units.inchesToMeters(12.0);    // TODO: measure
    public static final double kBackRightCamRoll  = 0.0;                           // TODO: measure
    public static final double kBackRightCamPitch = Units.degreesToRadians(-15.0); // TODO: measure
    public static final double kBackRightCamYaw   = Units.degreesToRadians(-135.0);// TODO: measure

    // ---- Intake Camera ----
    // Mounted below the slapdown intake, tilted downward to detect balls on the ground.
    // Does NOT do AprilTag detection — runs object-detection (ML) pipeline.
    public static final double kIntakeCamX     = Units.inchesToMeters(13.375); // TODO: measure (forward of center)
    public static final double kIntakeCamY     = 0.0;                          // TODO: measure
    public static final double kIntakeCamZ     = Units.inchesToMeters(6.0);    // TODO: measure
    public static final double kIntakeCamRoll  = 0.0;                          // TODO: measure
    public static final double kIntakeCamPitch = Units.degreesToRadians(45.0); // TODO: measure (positive = downward tilt)
    public static final double kIntakeCamYaw   = 0.0;                          // TODO: measure (facing forward)

    // ---- Side Camera ----
    // Mounted on the left side of the robot for lateral awareness.
    public static final double kSideCamX     = 0.0;                           // TODO: measure
    public static final double kSideCamY     = Units.inchesToMeters(11.375);  // TODO: measure (left side)
    public static final double kSideCamZ     = Units.inchesToMeters(10.0);    // TODO: measure
    public static final double kSideCamRoll  = 0.0;                           // TODO: measure
    public static final double kSideCamPitch = Units.degreesToRadians(-5.0);  // TODO: measure (slight upward tilt)
    public static final double kSideCamYaw   = Units.degreesToRadians(90.0);  // TODO: measure (facing left)

    // ==================== 16. VISION TUNING ====================

    /**
     * Pose-estimation standard deviations when MULTIPLE AprilTags are visible.
     * Format: { x_meters, y_meters, heading_radians }. Lower = more trust.
     */
    public static final double[] kMultiTagStdDevs = { 0.5, 0.5, 1.0 };

    /**
     * Pose-estimation standard deviations when only ONE AprilTag is visible.
     * Single-tag estimates are less reliable — values should be larger than multi-tag.
     */
    public static final double[] kSingleTagStdDevs = { 4.0, 4.0, 8.0 };

    /**
     * Aggressive multi-tag standard deviations used when we want to snap
     * odometry back to vision quickly after tags reappear.
     */
    public static final double[] kAggressiveMultiTagStdDevs = { 0.1, 0.1, 0.2 };

    // Ball-chase (VisionIntakeCommand) yaw PID
    public static final double kBallChaseYawP = 0.04;
    public static final double kBallChaseYawI = 0.0;
    public static final double kBallChaseYawD = 0.002;

    // ==================== 17. BALL DETECTION TUNING ====================

    /** Default spindexer motor current (amps) when spinning with empty hopper. */
    public static final double kDefaultSpindexerBaselineCurrent = 3.0;

    /** Default feeder motor current (amps) when spinning empty. */
    public static final double kDefaultFeederBaselineCurrent = 2.5;

    /**
     * Current delta above baseline (amps) to declare hopper has balls.
     * Tune on real robot: load 1 ball, observe delta, set above observed noise.
     */
    public static final double kSpindexerLoadedThreshold = 2.0;

    /**
     * Current delta above baseline (amps) below which hopper is considered empty.
     * Gap between this and kSpindexerLoadedThreshold is the hysteresis band.
     */
    public static final double kSpindexerEmptyThreshold = 0.5;

    /**
     * Current delta (amps) above baseline when hopper is completely full.
     * Used to normalize load level to 0.0–1.0.
     * TODO: measure with full hopper on real robot.
     */
    public static final double kSpindexerFullLoadDelta = 8.0;

    /**
     * Current delta (amps) above baseline indicating a ball is actively
     * being pushed into the flywheels by the feeder.
     * TODO: tune on real robot.
     */
    public static final double kFeederBallPresentThreshold = 3.0;

    // ==================== 18. SWERVE GEOMETRY ====================

    /**
     * Distance between the centers of the left and right swerve modules (track width),
     * in meters.
     *
     * <p>How to measure:
     * <ol>
     *   <li>Measure from the center of the front-left module axle to the center of the
     *       front-right module axle (or back-to-back — both pairs should be equal).
     *   <li>Convert to meters.
     * </ol>
     *
     * <p>Example: 22.75 inches = 0.5779 m.
     */
    public static final double kTrackWidthMeters = Units.inchesToMeters(22.75);

    /**
     * Distance between the centers of the front and rear swerve modules (wheelbase),
     * in meters.
     *
     * <p>How to measure:
     * <ol>
     *   <li>Measure from the center of the front-left module axle to the center of the
     *       back-left module axle (or front-right to back-right).
     *   <li>Convert to meters.
     * </ol>
     *
     * <p>Example: 22.75 inches = 0.5779 m.
     */
    public static final double kWheelBaseMeters = Units.inchesToMeters(22.75);

    // ==================== 19. CANCODER OFFSETS ====================
    //
    // Each CANcoder offset is the number of ROTATIONS to subtract from the raw reading
    // so that the wheel points straight forward when the encoder reads 0.
    //
    // How to calibrate ALL four encoders:
    //   1. Point all four wheels exactly straight forward (align by eye or with a straight edge).
    //   2. In Phoenix Tuner X, read the "Absolute Position" signal for each CANcoder.
    //   3. Enter that value (with sign) as the offset below.
    //   4. After setting offsets, verify: joystick forward → robot drives straight.
    //
    // Note: Phoenix 6 SwerveModule uses this as MagnetSensor.MagnetOffset.
    //   A positive offset rotates the zero point CCW (when viewed from the top of the encoder).

    /**
     * Front-Left CANcoder offset (rotations). Calibrate with wheel pointing straight forward.
     *
     * <p>TODO: calibrate on real robot
     */
    public static final double kFrontLeftEncoderOffset = 0.0; // rotations — calibrate!

    /**
     * Front-Right CANcoder offset (rotations). Calibrate with wheel pointing straight forward.
     *
     * <p>TODO: calibrate on real robot
     */
    public static final double kFrontRightEncoderOffset = 0.0; // rotations — calibrate!

    /**
     * Back-Left CANcoder offset (rotations). Calibrate with wheel pointing straight forward.
     *
     * <p>TODO: calibrate on real robot
     */
    public static final double kBackLeftEncoderOffset = 0.0; // rotations — calibrate!

    /**
     * Back-Right CANcoder offset (rotations). Calibrate with wheel pointing straight forward.
     *
     * <p>TODO: calibrate on real robot
     */
    public static final double kBackRightEncoderOffset = 0.0; // rotations — calibrate!
}
