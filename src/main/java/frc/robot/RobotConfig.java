package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Single source of truth for all per-robot hardware measurements.
 *
 * <p>When deploying to a new robot, only edit THIS file.
 * All other files (Constants.java, subsystems) read from here — no other files
 * need to change for a new robot deployment.
 *
 * <p>Sections:
 * <ol>
 *   <li>Shooter Geometry — position offset and exit height
 *   <li>Hood Calibration — motor encoder hard-stop readings
 *   <li>Turret Calibration — gear ratio, hard-stop readings, mount offset
 *   <li>Swerve Geometry — track width and wheelbase
 *   <li>CANcoder Offsets — absolute encoder zero-offset per module
 * </ol>
 */
public final class RobotConfig {

    private RobotConfig() {} // utility class

    // ==================== SHOOTER GEOMETRY ====================

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

    // ==================== HOOD CALIBRATION ====================

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
     * <p>Example: if Tuner reads 0.0 at minimum, use {@code 0.0}.
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
     *   <li>Power on the robot. Manually drive the hood motor to the maximum hard stop.
     *   <li>In Phoenix Tuner X, read the TalonFX position signal for the hood motor.
     *   <li>Record that value here (units: motor rotations).
     * </ol>
     *
     * <p>Example: if Tuner reads 5.0 at maximum, use {@code 5.0}.
     *
     * <p>TODO: calibrate on real robot
     */
    public static final double kHoodMaxMotorRotations = 0.0; // placeholder

    // ==================== TURRET CALIBRATION ====================

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
     * <p>Example: if the motor turns 100× per turret revolution, use {@code 100.0}.
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
     * <p>Example: if Tuner reads 27.78 motor rotations at CCW stop, use {@code 27.78}.
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
     * <p>Example: if Tuner reads -27.78 motor rotations at CW stop, use {@code -27.78}.
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

    // ==================== SWERVE GEOMETRY ====================

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

    // ==================== CANCODER OFFSETS ====================
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
