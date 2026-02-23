package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.ShooterConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.RobotState;
import frc.robot.ShooterPhysics;

import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * Fully automated shooter subsystem for FRC 2026 REBUILT.
 *
 * <h2>Philosophy: Track-on-demand.</h2>
 * The turret, hood, and flywheel only track the Hub when tracking is
 * explicitly enabled (driver holds shoot button or auto command requests it).
 * When tracking is disabled, all shooter motors idle and the turret homes.
 * <ul>
 *   <li>Turret → locks on to Hub when tracking enabled</li>
 *   <li>Hood → distance-based lookup table when tracking enabled</li>
 *   <li>Flywheel → distance-based lookup table when tracking enabled</li>
 * </ul>
 *
 * <p>The Feeder and Spindexer are separate subsystems that query
 * {@code isReadyToShoot()} and {@code getCompensatedDistance()} from this class.
 *
 * Works with AND without AprilTags (vision-fused odometry).
 * Shoot-while-moving compensation toggled via Constants.
 */

public class Shooter extends SubsystemBase {

    // ==================== SINGLETON ====================
    private static Shooter instance;

    public static void initialize(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        if (instance != null) throw new IllegalStateException("Shooter already initialized.");
        instance = new Shooter(poseSupplier, fieldSpeedsSupplier);
    }

    public static Shooter getInstance() {
        if (instance == null) throw new IllegalStateException("Shooter not initialized. Call initialize() first.");
        return instance;
    }

    public static void resetInstance() { instance = null; }

    // ==================== SUPPLIERS ====================
    private final Supplier<Pose2d> poseSupplier;
    private final Supplier<ChassisSpeeds> fieldSpeedsSupplier;

    // ==================== MOTORS ====================
    private final TalonFX flywheelMotor;
    private final TalonFX hoodMotor;
    private final TalonFX turretMotor;

    // ==================== CONTROL REQUESTS (reused to avoid GC) ====================
    // withUpdateFreqHz(50): Phoenix 6 default auto-resend is 100 Hz. 50 Hz matches the main
    // loop rate and halves CAN traffic on the RoboRIO bus from these three motors.
    private final VelocityVoltage flywheelRequest = new VelocityVoltage(0).withSlot(0).withUpdateFreqHz(50);
    private final PositionVoltage hoodRequest = new PositionVoltage(0).withSlot(0).withUpdateFreqHz(50);
    private final PositionVoltage turretRequest = new PositionVoltage(0).withSlot(0).withUpdateFreqHz(50);

    // ==================== CACHED STATUS SIGNALS ====================
    private final StatusSignal<AngularVelocity> flywheelVelocitySignal;
    private final StatusSignal<Angle> hoodPositionSignal;
    private final StatusSignal<Angle> turretPositionSignal;

    // ==================== HEALTH ====================
    private boolean healthy = true;
    private static final int kMaxConfigRetries = 5;

    // ==================== INTERPOLATION TABLES ====================
    private final InterpolatingDoubleTreeMap flywheelTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap hoodAngleTable = new InterpolatingDoubleTreeMap();
    private final InterpolatingDoubleTreeMap timeOfFlightTable = new InterpolatingDoubleTreeMap();

    // ==================== STATE ====================
    private double targetFlywheelRPS = 0.0;
    private double targetHoodAngleDeg = ShooterConstants.kHoodMinAngleDegrees;
    private double targetTurretAngleDeg = 0.0;

    // Cached auto-aim calculations (updated every periodic cycle)
    private double distanceToHub = 0.0;
    private double angleToHub = 0.0;
    private boolean inShootingRange = false;

    // Shoot-while-moving compensated values
    private double compensatedDistance = 0.0;
    private double compensatedTurretAngle = 0.0;
    private double flywheelCompensationRPS = 0.0;

    // ==================== TRACKING STATE ====================
    /** When false, shooter idles (no flywheel, hood home, turret home). */
    private boolean trackingEnabled = false;

    /** Alliance zone dump mode — fixed flywheel/hood/turret, no auto-aim. */
    private boolean allianceZoneMode = false;

    /** True when turret is near its rotation limit and drivetrain should help. */
    private boolean turretNearLimit = false;
    /** True when turret is AT its hard limit and target is beyond — emergency mode. */
    private boolean turretAtHardLimit = false;
    /** Proportional drivetrain rotation hint (rad/s) to help turret stay in range. */
    private double turretWraparoundHint = 0.0;

    // ==================== HOOD TRIM ====================
    /** Manual hood angle trim from operator D-pad (degrees, additive to auto-aim). */
    private double hoodTrimDeg = 0.0;

    // ==================== FLYWHEEL RECOVERY ====================
    /** Cooldown timer after a ball is shot to let flywheel recover speed. */
    private double lastBallShotTimestamp = 0.0;
    private static final double kFlywheelRecoverySeconds = ShooterConstants.kFlywheelRecoverySeconds;

    // Cached sensor values — refreshed once per periodic in batch
    private double cachedFlywheelVelocityRPS = 0.0;
    private double cachedHoodPositionDeg = 0.0;
    private double cachedTurretPositionDeg = 0.0;

    // ==================== RUNTIME HEALTH MONITORING ====================
    /** Consecutive cycles where flywheel velocity deviates significantly from target. */
    private int flywheelFaultCycles = 0;
    /** Consecutive cycles where turret position is stuck despite target change. */
    private int turretFaultCycles = 0;
    private double lastTurretTarget = 0.0;
    private double lastTurretPosition = 0.0;
    private static final int kFaultCycleThreshold = ShooterConstants.kFaultCycleThreshold;

    /**
     * @param poseSupplier        Supplies the current robot pose (from SwerveDrive::getPose).
     *                            Vision-fused — works with or without AprilTags.
     * @param fieldSpeedsSupplier Supplies the current field-relative chassis speeds
     *                            (from SwerveDrive::getFieldRelativeSpeeds).
     *                            Used for shoot-while-moving compensation.
     */
    private Shooter(Supplier<Pose2d> poseSupplier, Supplier<ChassisSpeeds> fieldSpeedsSupplier) {
        this.poseSupplier = poseSupplier;
        this.fieldSpeedsSupplier = fieldSpeedsSupplier;

        flywheelMotor = new TalonFX(ShooterConstants.kFlywheelMotorId, SwerveConstants.kCANivoreBus);
        hoodMotor     = new TalonFX(ShooterConstants.kHoodMotorId,     SwerveConstants.kCANivoreBus);
        turretMotor   = new TalonFX(ShooterConstants.kTurretMotorId); // rio CAN bus

        configureFlywheelMotor();
        configureHoodMotor();
        configureTurretMotor();
        populateShootingTables();

        // Cache status signals — read these instead of calling .getVelocity() every cycle
        flywheelVelocitySignal = flywheelMotor.getVelocity();
        hoodPositionSignal = hoodMotor.getPosition();
        turretPositionSignal = turretMotor.getPosition();

        // Flywheel at 50Hz for RPM recovery detection.
        // Turret at 50Hz — tracks moving target, needs responsive feedback.
        // Hood at 40Hz — adjusts by distance, slightly less time-critical than turret.
        flywheelVelocitySignal.setUpdateFrequency(50);
        hoodPositionSignal.setUpdateFrequency(40);
        turretPositionSignal.setUpdateFrequency(50);

        flywheelMotor.optimizeBusUtilization();
        hoodMotor.optimizeBusUtilization();
        turretMotor.optimizeBusUtilization();
    }

    /** Retry loop for config — handles transient CAN failures at startup. */
    private void applyConfig(java.util.function.Supplier<StatusCode> apply, String device) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < kMaxConfigRetries; i++) {
            status = apply.get();
            if (status.isOK()) return;
        }
        configHealthy = false;
        healthy = false;
        DriverStation.reportError("[Shooter] " + device + " config FAILED: " + status, false);
    }

    // ==================== MOTOR CONFIGURATION ====================

    private void configureFlywheelMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = ShooterConstants.kFlywheelP;
        config.Slot0.kI = ShooterConstants.kFlywheelI;
        config.Slot0.kD = ShooterConstants.kFlywheelD;
        config.Slot0.kS = ShooterConstants.kFlywheelS;
        config.Slot0.kV = ShooterConstants.kFlywheelV;
        config.Slot0.kA = ShooterConstants.kFlywheelA;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 80.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = ShooterConstants.kFlywheelGearRatio;

        applyConfig(() -> flywheelMotor.getConfigurator().apply(config), "Flywheel");
    }

    private void configureHoodMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = ShooterConstants.kHoodP;
        config.Slot0.kI = ShooterConstants.kHoodI;
        config.Slot0.kD = ShooterConstants.kHoodD;
        config.Slot0.kG = ShooterConstants.kHoodG;
        config.Slot0.GravityType = GravityTypeValue.Arm_Cosine;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = ShooterConstants.kHoodGearRatio;

        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants.kHoodMaxRotations;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants.kHoodMinRotations;

        applyConfig(() -> hoodMotor.getConfigurator().apply(config), "Hood");
        // IMPORTANT: This assumes the hood is physically at its minimum (most open) position
        // when the robot powers on. If the hood is not at min, all angle calculations will
        // be offset and soft limits will not protect the mechanism correctly.
        // A hard stop or known mechanical detent at the min position is required.
        hoodMotor.setPosition(ShooterConstants.kHoodMinRotations);
        DriverStation.reportWarning(
                "[Shooter] Hood encoder zeroed at min position — verify hood is physically at min!", false);
    }

    private void configureTurretMotor() {
        TalonFXConfiguration config = new TalonFXConfiguration();

        config.Slot0.kP = ShooterConstants.kTurretP;
        config.Slot0.kI = ShooterConstants.kTurretI;
        config.Slot0.kD = ShooterConstants.kTurretD;

        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 30.0;

        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        config.Feedback.SensorToMechanismRatio = ShooterConstants.kTurretGearRatio;

        // NOTE: ContinuousWrap is intentionally DISABLED for the turret.
        // ContinuousWrap + soft limits conflict — the wrap would fight the limits.
        // Limits are narrowed to ±175° to prevent oscillation at the wrap boundary.
        // Software clamp in setTurretAngle() enforces the same range.
        config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = ShooterConstants.kTurretMaxRotations;
        config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = ShooterConstants.kTurretMinRotations;

        config.ClosedLoopGeneral.ContinuousWrap = false;

        applyConfig(() -> turretMotor.getConfigurator().apply(config), "Turret");
        // IMPORTANT: This assumes the turret is physically at CENTER (0°) when the robot
        // powers on. If the turret is rotated to one side, all angle calculations will be
        // offset and the turret may collide with hard stops.
        // Ensure the turret has a mechanical index/detent at center before each match.
        turretMotor.setPosition(0);
        DriverStation.reportWarning(
                "[Shooter] Turret encoder zeroed at center (0°) — verify turret is physically centered!", false);
    }

    private void populateShootingTables() {
        // Compute tables from physics at init time — not during teleop/auto
        double[][] shootingTable = ShooterPhysics.computeShootingTable();
        double[][] tofTable = ShooterPhysics.computeTimeOfFlightTable();

        for (double[] entry : shootingTable) {
            flywheelTable.put(entry[0], entry[1]);
            hoodAngleTable.put(entry[0], entry[2]);
        }
        for (double[] entry : tofTable) {
            timeOfFlightTable.put(entry[0], entry[1]);
        }
        validateShootingTables(shootingTable);
    }

    /**
     * Validates shooting lookup tables at startup.
     * Checks that entries are monotonically increasing in distance,
     * flywheel RPS increases with distance, and hood angles are in bounds.
     */
    private void validateShootingTables(double[][] table) {
        for (int i = 0; i < table.length; i++) {
            double dist = table[i][0];
            double rps = table[i][1];
            double hood = table[i][2];

            // Check hood angle bounds
            if (hood < ShooterConstants.kHoodMinAngleDegrees || hood > ShooterConstants.kHoodMaxAngleDegrees) {
                DriverStation.reportWarning(
                        "[Shooter] Table entry " + i + ": hood angle " + hood
                                + "° is outside limits [" + ShooterConstants.kHoodMinAngleDegrees
                                + ", " + ShooterConstants.kHoodMaxAngleDegrees + "]", false);
            }

            // Check monotonicity
            if (i > 0) {
                if (dist <= table[i - 1][0]) {
                    DriverStation.reportError(
                            "[Shooter] Table entry " + i + ": distance " + dist
                                    + "m is not increasing from " + table[i - 1][0] + "m", false);
                }
                if (rps < table[i - 1][1]) {
                    DriverStation.reportWarning(
                            "[Shooter] Table entry " + i + ": flywheel RPS " + rps
                                    + " decreases from " + table[i - 1][1]
                                    + " — are you sure?", false);
                }
            }
        }
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        // ---- 0. Batch refresh all status signals ONCE per cycle ----
        // Flywheel and hood are on the CANivore bus; turret is on the rio bus.
        // refreshAll() requires all signals to share the same bus — split into two calls.
        com.ctre.phoenix6.BaseStatusSignal.refreshAll(flywheelVelocitySignal, hoodPositionSignal);
        com.ctre.phoenix6.BaseStatusSignal.refreshAll(turretPositionSignal);
        cachedFlywheelVelocityRPS = flywheelVelocitySignal.getValueAsDouble();
        cachedHoodPositionDeg = hoodPositionSignal.getValueAsDouble() * 360.0;
        cachedTurretPositionDeg = turretPositionSignal.getValueAsDouble() * 360.0;

        // ---- 1. Always compute aim solution so telemetry is available ----
        updateHubCalculations();

        // ---- 2. Track-on-demand / Alliance zone mode ----
        if (allianceZoneMode) {
            // Alliance zone dump: fixed parameters, no auto-aim
            setFlywheelRPS(ShooterConstants.kAllianceZoneFlywheelRPS);
            setHoodAngle(ShooterConstants.kAllianceZoneHoodAngleDeg);
            setTurretAngle(ShooterConstants.kAllianceZoneTurretAngleDeg);
            turretNearLimit = false;
            turretWraparoundHint = 0.0;
        } else if (!trackingEnabled) {
            // Tracking off: everything idles, turret homes to center
            setFlywheelRPS(0);
            setHoodAngle(ShooterConstants.kHoodMinAngleDegrees);
            setTurretAngle(0);
            turretNearLimit = false;
            turretWraparoundHint = 0.0;
        } else {
            // Tracking on: aim at Hub with brownout protection
            double voltComp = RobotState.getInstance().getVoltageCompensationFactor();
            boolean criticalBattery = RobotState.getInstance().isCriticalBattery();
            boolean brownoutPredicted = RobotState.getInstance().isBrownoutPredicted();

            // Turret wraparound detection with hysteresis:
            // Activates at kTurretWraparoundThresholdDeg, deactivates at kTurretWraparoundReleaseThresholdDeg.
            // Hint scales proportionally from 0 at the release threshold to max at the hard limit.
            double absTurretAngle = Math.abs(compensatedTurretAngle);
            double actualTurretAbs = Math.abs(cachedTurretPositionDeg);
            if (turretNearLimit) {
                // Already active — deactivate only when below release threshold (hysteresis)
                turretNearLimit = absTurretAngle > ShooterConstants.kTurretWraparoundReleaseThresholdDeg;
            } else {
                // Not active — activate when exceeding engage threshold
                turretNearLimit = absTurretAngle > ShooterConstants.kTurretWraparoundThresholdDeg;
            }

            // Hard limit detection: actual turret position is at the soft limit AND
            // the target is still beyond it. This means the turret physically cannot reach
            // the target — drivetrain MUST rotate or we lose tracking entirely.
            turretAtHardLimit = turretNearLimit
                    && actualTurretAbs > (ShooterConstants.kTurretMaxAngleDegrees - 3.0);

            if (turretNearLimit) {
                // Proportional hint: ramps from 0 at release threshold to max at hard limit
                double range = ShooterConstants.kTurretMaxAngleDegrees
                        - ShooterConstants.kTurretWraparoundReleaseThresholdDeg;
                double fraction = Math.min(
                        (absTurretAngle - ShooterConstants.kTurretWraparoundReleaseThresholdDeg) / range,
                        1.0);
                double hint = fraction * ShooterConstants.kTurretWraparoundMaxHintRadPerSec;

                // At hard limit: force maximum hint and block shooting readiness
                if (turretAtHardLimit) {
                    hint = ShooterConstants.kTurretWraparoundMaxHintRadPerSec;
                }

                turretWraparoundHint = Math.signum(compensatedTurretAngle) * hint;
            } else {
                turretWraparoundHint = 0.0;
                turretAtHardLimit = false;
            }

            if (criticalBattery) {
                // Critical battery: stop flywheel to preserve power for drivetrain
                setFlywheelRPS(0);
                setHoodAngle(ShooterConstants.kHoodMinAngleDegrees);
                setTurretAngle(compensatedTurretAngle);
            } else if (brownoutPredicted) {
                // Brownout imminent: reduce flywheel speed proactively
                if (inShootingRange) {
                    // Compute scaled flywheel speed first, then send single command
                    double baseRPS = flywheelTable.get(compensatedDistance) + flywheelCompensationRPS;
                    setFlywheelRPS(baseRPS * 0.7);
                    setHoodFromDistance(compensatedDistance);
                    setTurretAngle(compensatedTurretAngle);
                } else {
                    setFlywheelRPS(0); // no pre-spin during predicted brownout
                    setHoodAngle(ShooterConstants.kHoodMinAngleDegrees);
                    setTurretAngle(compensatedTurretAngle);
                }
            } else if (inShootingRange) {
                setFlywheelFromDistance(compensatedDistance);
                setHoodFromDistance(compensatedDistance);
                setTurretAngle(compensatedTurretAngle);
            } else {
                // Out of range: idle flywheel at a low pre-spin, hood to minimum, turret still tracks
                setFlywheelRPS(ShooterConstants.kIdleFlywheelRPS * voltComp);
                setHoodAngle(ShooterConstants.kHoodMinAngleDegrees);
                setTurretAngle(compensatedTurretAngle);
            }
        }

        // ---- 3. Telemetry (AdvantageKit only — reduces CAN/NT traffic) ----
        Logger.recordOutput("Shooter/DistanceToHub", distanceToHub);
        Logger.recordOutput("Shooter/AngleToHub", angleToHub);
        Logger.recordOutput("Shooter/CompensatedDistance", compensatedDistance);
        Logger.recordOutput("Shooter/CompTurretAngle", compensatedTurretAngle);
        Logger.recordOutput("Shooter/FlywheelCompRPS", flywheelCompensationRPS);
        Logger.recordOutput("Shooter/InRange", inShootingRange);
        Logger.recordOutput("Shooter/FlywheelTargetRPS", targetFlywheelRPS);
        Logger.recordOutput("Shooter/FlywheelActualRPS", cachedFlywheelVelocityRPS);
        Logger.recordOutput("Shooter/FlywheelAtSpeed", isFlywheelAtSpeed());
        Logger.recordOutput("Shooter/HoodTargetDeg", targetHoodAngleDeg);
        Logger.recordOutput("Shooter/HoodActualDeg", cachedHoodPositionDeg);
        Logger.recordOutput("Shooter/HoodAtTarget", isHoodAtTarget());
        Logger.recordOutput("Shooter/TurretTargetDeg", targetTurretAngleDeg);
        Logger.recordOutput("Shooter/TurretActualDeg", cachedTurretPositionDeg);
        Logger.recordOutput("Shooter/TurretAtTarget", isTurretAtTarget());
        Logger.recordOutput("Shooter/TurretSpringFF",
                ShooterConstants.kTurretSpringFeedForwardVPerDeg * cachedTurretPositionDeg);
        Logger.recordOutput("Shooter/TrackingEnabled", trackingEnabled);
        Logger.recordOutput("Shooter/AllianceZoneMode", allianceZoneMode);
        Logger.recordOutput("Shooter/ReadyToShoot", isReadyToShoot());
        Logger.recordOutput("Shooter/TurretNearLimit", turretNearLimit);
        Logger.recordOutput("Shooter/TurretAtHardLimit", turretAtHardLimit);
        Logger.recordOutput("Shooter/TurretWraparoundHint", turretWraparoundHint);
        Logger.recordOutput("Shooter/HoodTrimDeg", hoodTrimDeg);

        // ---- 4. Runtime health monitoring ----
        monitorRuntimeHealth();

        // ---- 3D Visualization for AdvantageScope ----
        // Turret pose: rotates around Z axis at robot center
        Pose2d robotPose = poseSupplier.get();
        double turretFieldYaw = robotPose.getRotation().getRadians()
                + Math.toRadians(cachedTurretPositionDeg);
        Logger.recordOutput("Shooter/TurretPose3d",
                new edu.wpi.first.math.geometry.Pose3d(
                        robotPose.getX(), robotPose.getY(), ShooterConstants.kTurretHeightMeters,
                        new edu.wpi.first.math.geometry.Rotation3d(0, 0, turretFieldYaw)));

        // Hood pose: tilts on the turret (pitch axis)
        double hoodPitchRad = Math.toRadians(cachedHoodPositionDeg);
        Logger.recordOutput("Shooter/HoodPose3d",
                new edu.wpi.first.math.geometry.Pose3d(
                        robotPose.getX(), robotPose.getY(), ShooterConstants.kHoodHeightMeters,
                        new edu.wpi.first.math.geometry.Rotation3d(0, -hoodPitchRad, turretFieldYaw)));

        // Combined Pose3d array for AdvantageScope 3D mechanism visualization
        Logger.recordOutput("Mechanism3d/Shooter", new edu.wpi.first.math.geometry.Pose3d[]{
                new edu.wpi.first.math.geometry.Pose3d(
                        robotPose.getX(), robotPose.getY(), ShooterConstants.kTurretHeightMeters,
                        new edu.wpi.first.math.geometry.Rotation3d(0, 0, turretFieldYaw)),
                new edu.wpi.first.math.geometry.Pose3d(
                        robotPose.getX(), robotPose.getY(), ShooterConstants.kHoodHeightMeters,
                        new edu.wpi.first.math.geometry.Rotation3d(0, -hoodPitchRad, turretFieldYaw))
        });

        // Health reporting
        RobotState.getInstance().setShooterHealthy(healthy);
    }

    // ==================== AUTO-AIM CALCULATIONS ====================

    /**
     * Calculates the distance and turret angle to the Hub from the current shooter position,
     * with shoot-while-moving compensation using distance-based time-of-flight.
     *
     * <p>The shooter position is the robot odometry center plus a configurable offset
     * (kShooterPositionOffset) rotated into field frame. When the offset is zero this
     * reduces to the original robot-center calculation.
     *
     * <p>Shoot-while-moving accounts for:
     * <ul>
     *   <li>Future shooter position when the robot translates during ball flight</li>
     *   <li>Future shooter position when the robot rotates during ball flight (offset rotates)</li>
     *   <li>Shooter's true field velocity = robot velocity + ω × shooter offset</li>
     * </ul>
     */
    private void updateHubCalculations() {
        Pose2d robotPose = poseSupplier.get();
        Translation2d robotTranslation = robotPose.getTranslation();
        double robotHeadingRad = robotPose.getRotation().getRadians();

        // ===== STEP 1: Compute shooter position in field frame =====
        // Rotate the robot-frame offset by the current robot heading.
        Translation2d shooterOffsetField = ShooterConstants.kShooterPositionOffset
                .rotateBy(robotPose.getRotation());
        Translation2d shooterTranslation = robotTranslation.plus(shooterOffsetField);

        // ===== STATIC AIM (from actual shooter position) =====
        Translation2d hubPosition = ShooterConstants.getActiveHubPosition();
        Translation2d shooterToHub = hubPosition.minus(shooterTranslation);
        distanceToHub = shooterToHub.getNorm();

        inShootingRange = distanceToHub >= ShooterConstants.kMinShootingDistanceMeters
                       && distanceToHub <= ShooterConstants.kMaxShootingDistanceMeters;

        double fieldAngleToHubRad = Math.atan2(shooterToHub.getY(), shooterToHub.getX());
        double turretAngleRad = fieldAngleToHubRad - robotHeadingRad;

        angleToHub = Math.toDegrees(MathUtil.angleModulus(turretAngleRad))
                   + ShooterConstants.kTurretMountOffsetDegrees;

        // ===== SHOOT-WHILE-MOVING COMPENSATION =====
        if (ShooterConstants.kShootWhileMovingEnabled) {
            ChassisSpeeds fieldSpeeds = fieldSpeedsSupplier.get();
            double vx = fieldSpeeds.vxMetersPerSecond;
            double vy = fieldSpeeds.vyMetersPerSecond;
            double omega = fieldSpeeds.omegaRadiansPerSecond;

            // Distance-based time-of-flight from lookup table
            double tof = timeOfFlightTable.get(distanceToHub);

            // Future robot position after ball flight time
            double futureRobotX = robotTranslation.getX() + vx * tof;
            double futureRobotY = robotTranslation.getY() + vy * tof;

            // Future robot heading (robot rotates during flight)
            double futureHeadingRad = robotHeadingRad + omega * tof;

            // Future shooter offset (rotated by future heading, since offset is in robot frame)
            Translation2d futureShooterOffset = ShooterConstants.kShooterPositionOffset
                    .rotateBy(new Rotation2d(futureHeadingRad));

            // Future shooter position in field frame
            Translation2d futureShooterPos = new Translation2d(
                    futureRobotX + futureShooterOffset.getX(),
                    futureRobotY + futureShooterOffset.getY());

            Translation2d futureToHub = hubPosition.minus(futureShooterPos);
            compensatedDistance = futureToHub.getNorm();

            double compensatedFieldAngle = Math.atan2(futureToHub.getY(), futureToHub.getX());
            double compensatedTurretRad = compensatedFieldAngle - robotHeadingRad;
            compensatedTurretAngle = Math.toDegrees(MathUtil.angleModulus(compensatedTurretRad))
                                   + ShooterConstants.kTurretMountOffsetDegrees;

            // Shooter's true field velocity = robot velocity + ω × shooter_offset_field
            // Cross product in 2D: ω × (rx, ry) = (-ω·ry, ω·rx)
            double vShooterX = vx - omega * shooterOffsetField.getY();
            double vShooterY = vy + omega * shooterOffsetField.getX();

            // Velocity component toward hub (dot product with hub direction unit vector)
            double hubAngle = Math.atan2(shooterToHub.getY(), shooterToHub.getX());
            double vTowardHub = vShooterX * Math.cos(hubAngle) + vShooterY * Math.sin(hubAngle);

            // Convert ground-speed compensation to flywheel RPS:
            // v_exit = RPS × circumference × efficiency  →  delta_RPS = -v / (C × η)
            flywheelCompensationRPS = -vTowardHub
                    / (ShooterConstants.kBottomFlywheelCircumferenceMeters
                       * ShooterConstants.kShooterEfficiencyFactor);
        } else {
            compensatedDistance = distanceToHub;
            compensatedTurretAngle = angleToHub;
            flywheelCompensationRPS = 0.0;
        }

        // Telemetry: shooter field pose for AdvantageScope visualization
        Logger.recordOutput("Shooter/ShooterPoseField",
                new Pose2d(shooterTranslation, robotPose.getRotation()));
    }

    /** True if initial motor configuration succeeded. Never cleared once set to false. */
    private boolean configHealthy = true;

    /** Whether the startup homing check has been performed. */
    private boolean homingChecked = false;

    /**
     * One-time check on first tracking enable: verify that turret and hood are
     * near their expected startup positions. If they're far off, the encoder
     * zero was wrong (mechanism was physically displaced at power-on).
     */
    private void checkHomingOnFirstEnable() {
        if (homingChecked) return;
        homingChecked = true;

        // After setPosition(0) in constructor, turret should read near 0°.
        // If it reads > 30° away, the physical position didn't match the assumed zero.
        if (Math.abs(cachedTurretPositionDeg) > 30.0) {
            DriverStation.reportError(
                    "[Shooter] HOMING ERROR: Turret reads " + String.format("%.1f°", cachedTurretPositionDeg)
                    + " but was zeroed at 0°. Physical position does not match encoder! "
                    + "Soft limits may be WRONG — risk of hard stop collision.", false);
        }

        // Hood should read near kHoodMinAngleDegrees after setPosition(kHoodMinRotations).
        double expectedHoodDeg = ShooterConstants.kHoodMinAngleDegrees;
        if (Math.abs(cachedHoodPositionDeg - expectedHoodDeg) > 15.0) {
            DriverStation.reportError(
                    "[Shooter] HOMING ERROR: Hood reads " + String.format("%.1f°", cachedHoodPositionDeg)
                    + " but was zeroed at " + String.format("%.1f°", expectedHoodDeg)
                    + ". Physical position does not match encoder!", false);
        }
    }

    /**
     * Detects runtime faults in the shooter beyond initial configuration failures.
     * Catches issues like flywheel motor failure mid-match or jammed turret.
     */
    private void monitorRuntimeHealth() {
        if (!configHealthy) {
            healthy = false;
            return; // config failed at startup — stay unhealthy forever
        }

        if (!trackingEnabled) {
            flywheelFaultCycles = 0;
            turretFaultCycles = 0;
            return;
        }

        // Flywheel fault: target > 10 RPS but actual < 10% of target for too long
        if (targetFlywheelRPS > 10.0) {
            double ratio = cachedFlywheelVelocityRPS / targetFlywheelRPS;
            if (ratio < 0.1) {
                flywheelFaultCycles++;
            } else {
                flywheelFaultCycles = Math.max(0, flywheelFaultCycles - 2);
            }
        } else {
            flywheelFaultCycles = 0;
        }

        // Turret fault: target changed significantly but position hasn't moved
        double turretTargetDelta = Math.abs(targetTurretAngleDeg - lastTurretTarget);
        double turretPositionDelta = Math.abs(cachedTurretPositionDeg - lastTurretPosition);
        if (turretTargetDelta > 5.0 && turretPositionDelta < 0.5) {
            turretFaultCycles++;
        } else {
            turretFaultCycles = Math.max(0, turretFaultCycles - 2);
        }
        lastTurretTarget = targetTurretAngleDeg;
        lastTurretPosition = cachedTurretPositionDeg;

        // Report faults
        if (flywheelFaultCycles >= kFaultCycleThreshold) {
            healthy = false;
            DriverStation.reportWarning("[Shooter] FLYWHEEL FAULT: motor not responding", false);
        } else if (turretFaultCycles >= kFaultCycleThreshold) {
            healthy = false;
            DriverStation.reportWarning("[Shooter] TURRET FAULT: position not tracking target", false);
        } else {
            // Clear fault once both counters are below threshold (recovery path)
            healthy = true;
        }
    }

    // ==================== GETTERS ====================

    /** @return horizontal distance to Hub in meters */
    public double getDistanceToHub() {
        return distanceToHub;
    }

    /** @return compensated distance to Hub (accounting for robot motion) */
    public double getCompensatedDistance() {
        return compensatedDistance;
    }

    /** @return robot-relative turret angle to Hub in degrees */
    public double getAngleToHub() {
        return angleToHub;
    }

    /** @return true if robot is within valid shooting range */
    public boolean isInShootingRange() {
        return inShootingRange;
    }

    // ==================== FLYWHEEL CONTROL ====================

    public void setFlywheelRPS(double rps) {
        targetFlywheelRPS = rps;
        flywheelMotor.setControl(flywheelRequest.withVelocity(rps));
    }

    public void setFlywheelFromDistance(double distanceMeters) {
        setFlywheelRPS(flywheelTable.get(distanceMeters) + flywheelCompensationRPS);
    }

    public void stopFlywheel() {
        targetFlywheelRPS = 0;
        flywheelMotor.stopMotor();
    }

    public double getFlywheelVelocityRPS() {
        return cachedFlywheelVelocityRPS;
    }

    /** @return the current target flywheel RPS (0 when not tracking) */
    public double getTargetFlywheelRPS() {
        return targetFlywheelRPS;
    }

    public double getFlywheelSurfaceSpeedMPS() {
        return cachedFlywheelVelocityRPS * ShooterConstants.kBottomFlywheelCircumferenceMeters;
    }

    public boolean isFlywheelAtSpeed() {
        if (targetFlywheelRPS == 0) return false;
        return Math.abs(getFlywheelVelocityRPS() - targetFlywheelRPS)
                < ShooterConstants.kFlywheelToleranceRPS;
    }

    // ==================== HOOD CONTROL ====================

    public void setHoodAngle(double angleDegrees) {
        targetHoodAngleDeg = MathUtil.clamp(
                angleDegrees,
                ShooterConstants.kHoodMinAngleDegrees,
                ShooterConstants.kHoodMaxAngleDegrees);
        hoodMotor.setControl(hoodRequest.withPosition(targetHoodAngleDeg / 360.0));
    }

    public void setHoodFromDistance(double distanceMeters) {
        setHoodAngle(hoodAngleTable.get(distanceMeters) + hoodTrimDeg);
    }

    public double getHoodAngleDegrees() {
        return cachedHoodPositionDeg;
    }

    public boolean isHoodAtTarget() {
        return Math.abs(getHoodAngleDegrees() - targetHoodAngleDeg)
                < ShooterConstants.kHoodToleranceDegrees;
    }

    /**
     * Adjust the hood angle trim by delta degrees. Clamped to ±4°.
     * Used by operator D-pad for mid-match correction.
     */
    public void adjustHoodTrim(double deltaDeg) {
        hoodTrimDeg = MathUtil.clamp(hoodTrimDeg + deltaDeg, -4.0, 4.0);
    }

    /** @return current hood trim in degrees */
    public double getHoodTrimDeg() {
        return hoodTrimDeg;
    }

    // ==================== TURRET CONTROL ====================

    public void setTurretAngle(double angleDegrees) {
        targetTurretAngleDeg = MathUtil.clamp(
                angleDegrees,
                ShooterConstants.kTurretMinAngleDegrees,
                ShooterConstants.kTurretMaxAngleDegrees);
        // Spring cable feedforward: the Vulcan spring exerts a restoring torque proportional
        // to the turret's displacement from center (0°). This compensates so the PID doesn't
        // have to fight the spring — preventing drift when holding off-center positions.
        // Tune kTurretSpringFeedForwardVPerDeg in RobotConfig until drift stops.
        double springFF = ShooterConstants.kTurretSpringFeedForwardVPerDeg * cachedTurretPositionDeg;
        turretMotor.setControl(turretRequest
                .withPosition(targetTurretAngleDeg / 360.0)
                .withFeedForward(springFF));
    }

    public double getTurretAngleDegrees() {
        return cachedTurretPositionDeg;
    }

    public boolean isTurretAtTarget() {
        return Math.abs(getTurretAngleDegrees() - targetTurretAngleDeg)
                < ShooterConstants.kTurretToleranceDegrees;
    }

    // ==================== READY CHECK ====================

    /**
     * Returns true when all systems are locked on and the feeder can send a ball.
     * Flywheel at speed + hood at angle + turret aimed + in shooting range +
     * flywheel has recovered from last shot.
     */
    public boolean isReadyToShoot() {
        if (!healthy) return false; // don't feed through a faulted shooter
        if (allianceZoneMode) {
            // Alliance zone mode: ready when flywheel + hood + turret are at fixed setpoints
            boolean flywheelRecovered =
                    (Timer.getFPGATimestamp() - lastBallShotTimestamp) >= kFlywheelRecoverySeconds;
            return isFlywheelAtSpeed() && isHoodAtTarget() && isTurretAtTarget() && flywheelRecovered;
        }
        if (!trackingEnabled) return false;
        // Turret at hard limit — cannot aim at target, do NOT feed.
        // Wait for drivetrain to rotate via wraparound hint.
        if (turretAtHardLimit) return false;
        boolean flywheelRecovered =
                (Timer.getFPGATimestamp() - lastBallShotTimestamp) >= kFlywheelRecoverySeconds;
        return isFlywheelAtSpeed() && isHoodAtTarget() && isTurretAtTarget()
                && inShootingRange && flywheelRecovered;
    }

    /**
     * Notify the shooter that a ball was just shot (beam break triggered).
     * Starts the flywheel recovery cooldown timer.
     * Called by Superstructure when a ball passes through the feeder beam break.
     */
    public void notifyBallShot() {
        lastBallShotTimestamp = Timer.getFPGATimestamp();
    }

    /**
     * Stops all shooter motors.
     */
    public void stopAll() {
        stopFlywheel();
        hoodMotor.stopMotor();
        turretMotor.stopMotor();
    }

    // ==================== TRACKING CONTROL ====================

    /** Enable Hub tracking — flywheel spins up, turret + hood aim at Hub. */
    public void enableTracking() {
        trackingEnabled = true;
        allianceZoneMode = false; // mutually exclusive
        checkHomingOnFirstEnable();
    }

    /** Disable Hub tracking — flywheel stops, turret + hood home. */
    public void disableTracking() {
        trackingEnabled = false;
    }

    /** @return true if tracking is currently enabled */
    public boolean isTrackingEnabled() {
        return trackingEnabled;
    }

    /** @return true if all shooter hardware is healthy */
    public boolean isHealthy() {
        return healthy;
    }

    /**
     * Enable alliance zone dump mode — fixed flywheel/hood/turret for lobbing
     * FUEL from the neutral zone into the alliance zone. No auto-aim.
     */
    public void enableAllianceZoneMode() {
        allianceZoneMode = true;
        trackingEnabled = false; // mutually exclusive
    }

    /** Disable alliance zone dump mode. */
    public void disableAllianceZoneMode() {
        allianceZoneMode = false;
    }

    /** @return true if alliance zone dump mode is active */
    public boolean isAllianceZoneMode() {
        return allianceZoneMode;
    }

    /** @return true if turret is near its rotation limit and drivetrain should help rotate. */
    public boolean isTurretNearLimit() {
        return turretNearLimit;
    }

    /** @return true if turret is physically at its hard limit and cannot reach the target. */
    public boolean isTurretAtHardLimit() {
        return turretAtHardLimit;
    }

    /**
     * @return rotation hint for the drivetrain when turret is near limit.
     *         Positive = turret is near positive limit (drivetrain should rotate CCW).
     *         Negative = turret is near negative limit (drivetrain should rotate CW).
     *         Zero = turret is fine, no drivetrain assistance needed.
     */
    public double getTurretWraparoundHint() {
        return turretWraparoundHint;
    }

    // ==================== COMMAND FACTORIES ====================

    /**
     * Waits until shooter is fully locked on and ready to fire.
     * Tracking must be enabled before calling this.
     * Useful in auto sequences: {@code waitUntilReady().andThen(feeder.autoFeedCommand(2))}.
     */
    public Command waitUntilReady() {
        return Commands.waitUntil(this::isReadyToShoot)
                .withName("Wait Until Ready");
    }

    // ==================== SIMULATION SUPPORT ====================

    /** Exposes flywheel TalonFX sim state for physics simulation. */
    public com.ctre.phoenix6.sim.TalonFXSimState getFlywheelSimState() {
        return flywheelMotor.getSimState();
    }

    /** Exposes hood TalonFX sim state for physics simulation. */
    public com.ctre.phoenix6.sim.TalonFXSimState getHoodSimState() {
        return hoodMotor.getSimState();
    }

    /** Exposes turret TalonFX sim state for physics simulation. */
    public com.ctre.phoenix6.sim.TalonFXSimState getTurretSimState() {
        return turretMotor.getSimState();
    }

    // ==================== SYSID CHARACTERIZATION ====================

    /** Apply raw voltage to the flywheel motor (for SysId). */
    public void setFlywheelVoltage(double volts) {
        flywheelMotor.setVoltage(volts);
    }

    /** Apply raw voltage to the hood motor (for SysId). */
    public void setHoodVoltage(double volts) {
        hoodMotor.setVoltage(volts);
    }

    /** Apply raw voltage to the turret motor (for SysId). */
    public void setTurretVoltage(double volts) {
        turretMotor.setVoltage(volts);
    }

    /** Hood position in rotations (for SysId logging). */
    public double getHoodPositionRotations() {
        return cachedHoodPositionDeg / 360.0;
    }

    /** Turret position in rotations (for SysId logging). */
    public double getTurretPositionRotations() {
        return cachedTurretPositionDeg / 360.0;
    }
}
