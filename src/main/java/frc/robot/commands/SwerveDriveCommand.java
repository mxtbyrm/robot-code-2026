package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveDrive;

/**
 * Einstein-grade teleop swerve drive command with full driver assistance.
 *
 * <h2>Features:</h2>
 * <ul>
 *   <li><b>Slew rate limiting</b> — prevents wheel slip from sudden input changes</li>
 *   <li><b>Squared inputs</b> — finer low-speed control while maintaining max speed</li>
 *   <li><b>Deadband</b> — prevents drift from non-centered joysticks</li>
 *   <li><b>Heading lock</b> — automatically holds current heading when rotation stick
 *       is released (ProfiledPIDController with trapezoidal profile)</li>
 *   <li><b>Slow mode</b> — reduces max speed while trigger is held for fine positioning</li>
 *   <li><b>Snap-to-angle</b> — driver can snap heading to cardinal directions</li>
 * </ul>
 */
public class SwerveDriveCommand extends Command {

    private final SwerveDrive swerveDrive;
    private final DoubleSupplier xSupplier;
    private final DoubleSupplier ySupplier;
    private final DoubleSupplier rotSupplier;
    private final BooleanSupplier slowModeSupplier;
    private final BooleanSupplier intakingSupplier;

    // Slew rate limiters — prevent instantaneous speed changes (reduces wheel slip)
    private final SlewRateLimiter xLimiter =
            new SlewRateLimiter(SwerveConstants.kTranslationSlewRate);
    private final SlewRateLimiter yLimiter =
            new SlewRateLimiter(SwerveConstants.kTranslationSlewRate);
    private final SlewRateLimiter rotLimiter =
            new SlewRateLimiter(SwerveConstants.kRotationSlewRate);

    // Heading lock PID — holds heading when driver releases rotation stick
    private final ProfiledPIDController headingController = new ProfiledPIDController(
            SwerveConstants.kHeadingLockP,
            SwerveConstants.kHeadingLockI,
            SwerveConstants.kHeadingLockD,
            new TrapezoidProfile.Constraints(
                    SwerveConstants.kMaxAngularSpeedRadiansPerSecond,
                    SwerveConstants.kMaxAngularAccelRadiansPerSecondSq));

    private boolean headingLocked = false;
    private double headingSetpoint = 0.0;

    // Snap-to-angle
    private Double snapAngle = null;

    // Turret wraparound assist — helps turret by rotating drivetrain
    private Shooter shooter = null;

    /**
     * @param swerveDrive      The swerve drivetrain subsystem
     * @param xSupplier        Supplies forward/backward speed (forward positive)
     * @param ySupplier        Supplies left/right speed (left positive)
     * @param rotSupplier      Supplies rotation speed (CCW positive)
     * @param slowModeSupplier Supplies whether slow mode is active
     * @param intakingSupplier Supplies whether robot is actively intaking (caps chassis speed)
     */
    public SwerveDriveCommand(
            SwerveDrive swerveDrive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier,
            BooleanSupplier slowModeSupplier,
            BooleanSupplier intakingSupplier) {

        this.swerveDrive = swerveDrive;
        this.xSupplier = xSupplier;
        this.ySupplier = ySupplier;
        this.rotSupplier = rotSupplier;
        this.slowModeSupplier = slowModeSupplier;
        this.intakingSupplier = intakingSupplier;

        headingController.enableContinuousInput(-Math.PI, Math.PI);
        headingController.setTolerance(Math.toRadians(SwerveConstants.kHeadingLockToleranceDeg));

        addRequirements(swerveDrive);
    }

    /** Backward-compatible constructor without slow mode or intake limiting. */
    public SwerveDriveCommand(
            SwerveDrive swerveDrive,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier rotSupplier) {
        this(swerveDrive, xSupplier, ySupplier, rotSupplier, () -> false, () -> false);
    }

    /**
     * Provide a reference to the Shooter for turret wraparound assistance.
     * When the turret approaches its rotation limit, the drivetrain auto-rotates
     * to bring the target back toward turret center.
     */
    public void setShooterReference(Shooter shooter) {
        this.shooter = shooter;
    }

    @Override
    public void initialize() {
        headingLocked = false;
        snapAngle = null;
    }

    @Override
    public void execute() {
        // Get joystick values with deadband
        double xSpeed = MathUtil.applyDeadband(xSupplier.getAsDouble(), OperatorConstants.kDeadband);
        double ySpeed = MathUtil.applyDeadband(ySupplier.getAsDouble(), OperatorConstants.kDeadband);
        double rotSpeed = MathUtil.applyDeadband(rotSupplier.getAsDouble(), OperatorConstants.kDeadband);

        // Square the inputs for finer control at low speeds while preserving sign
        xSpeed = Math.copySign(xSpeed * xSpeed, xSpeed);
        ySpeed = Math.copySign(ySpeed * ySpeed, ySpeed);
        rotSpeed = Math.copySign(rotSpeed * rotSpeed, rotSpeed);

        // Slow mode — fine positioning near scoring locations or during defense
        boolean slowMode = slowModeSupplier.getAsBoolean();
        double speedMultiplier = slowMode
                ? SwerveConstants.kSlowModeSpeedMultiplier
                : 1.0;
        double rotMultiplier = slowMode
                ? SwerveConstants.kSlowModeRotMultiplier
                : 1.0;

        // Scale to max speeds (with slow mode factor)
        xSpeed *= SwerveConstants.kMaxSpeedMetersPerSecond * speedMultiplier;
        ySpeed *= SwerveConstants.kMaxSpeedMetersPerSecond * speedMultiplier;
        rotSpeed *= SwerveConstants.kMaxAngularSpeedRadiansPerSecond * rotMultiplier;

        // Slew rate limit — prevents sudden acceleration that causes wheel slip.
        // When the translation stick is released (both axes zero after deadband),
        // reset the limiters to a small fraction of their current output instead
        // of maintaining full ramp-down (which produces meaningless 0° module angles).
        // This gives the VelocityVoltage PID a brief transition window to decelerate
        // smoothly while still cutting off lingering non-zero ChassisSpeeds quickly.
        if (xSpeed == 0.0 && ySpeed == 0.0) {
            // Reset to 10% of current output — provides 1-2 frames of decel ramp
            // before the next cycle sees zero input and resets again.
            xLimiter.reset(xLimiter.lastValue() * 0.1);
            yLimiter.reset(yLimiter.lastValue() * 0.1);
            // Still command zero — the limiter will ramp from 10% toward 0
            xSpeed = xLimiter.calculate(0);
            ySpeed = yLimiter.calculate(0);
        } else {
            xSpeed = xLimiter.calculate(xSpeed);
            ySpeed = yLimiter.calculate(ySpeed);
        }
        rotSpeed = rotLimiter.calculate(rotSpeed);

        // ==================== INTAKE SPEED LIMIT ====================
        // While actively intaking, cap chassis speed below roller surface speed.
        // If chassis moves faster than rollers spin, balls get pushed away
        // instead of being pulled in — this prevents that.
        if (intakingSupplier.getAsBoolean()) {
            double maxIntakeSpeed = IntakeConstants.kMaxChassisSpeedWhileIntaking;
            double translationMag = Math.hypot(xSpeed, ySpeed);
            if (translationMag > maxIntakeSpeed) {
                double scale = maxIntakeSpeed / translationMag;
                xSpeed *= scale;
                ySpeed *= scale;
            }
        }

        // ==================== HEADING LOCK / SNAP-TO-ANGLE ====================
        double currentHeading = swerveDrive.getHeading().getRadians();

        // Get turret wraparound hint (proportional, rad/s).
        double wraparoundHint = 0.0;
        boolean turretAtHardLimit = false;
        if (shooter != null && shooter.isTurretNearLimit()) {
            wraparoundHint = shooter.getTurretWraparoundHint();
            turretAtHardLimit = shooter.isTurretAtHardLimit();
        }

        if (snapAngle != null) {
            // Snap-to-angle mode: PID drives rotation to snap target
            rotSpeed = headingController.calculate(currentHeading, snapAngle);
            if (headingController.atGoal()) {
                headingSetpoint = snapAngle;
                snapAngle = null;
                headingLocked = true;
            }
        } else if (Math.abs(rotSupplier.getAsDouble()) > OperatorConstants.kDeadband) {
            // Driver is commanding rotation — release heading lock
            headingLocked = false;
        } else if (!headingLocked) {
            // Driver just released rotation stick — capture heading
            headingSetpoint = currentHeading;
            headingController.reset(currentHeading);
            headingLocked = true;
        }

        if (headingLocked && snapAngle == null) {
            if (turretAtHardLimit) {
                // Turret at hard limit — override heading lock entirely.
                // The turret physically cannot reach the target, so the drivetrain
                // MUST rotate. Continuously update the setpoint so heading lock
                // doesn't fight back on the next cycle.
                headingSetpoint = currentHeading;
                headingController.reset(currentHeading);
                rotSpeed = wraparoundHint;
            } else {
                // Normal heading lock: hold captured heading with PID, add hint on top
                double correction = headingController.calculate(currentHeading, headingSetpoint);
                rotSpeed = correction + wraparoundHint;
            }
        } else {
            // Not heading locked (driver commanding rotation) — add hint to driver input
            rotSpeed += wraparoundHint;
        }

        // Drive field-relative
        swerveDrive.drive(xSpeed, ySpeed, rotSpeed, true);

        // Telemetry
        Logger.recordOutput("Drive/HeadingLocked", headingLocked);
        Logger.recordOutput("Drive/SlowMode", slowMode);
        if (snapAngle != null) {
            Logger.recordOutput("Drive/SnapTarget", Math.toDegrees(snapAngle));
        }
    }

    /**
     * Snap the robot heading to a specific angle (degrees, field-relative).
     * Call from a button binding.
     *
     * @param angleDegrees Target heading in degrees (0 = away from driver station)
     */
    public void snapToAngle(double angleDegrees) {
        snapAngle = Math.toRadians(angleDegrees);
        headingController.reset(swerveDrive.getHeading().getRadians());
    }

    /** Cancel any active snap-to-angle. */
    public void cancelSnap() {
        snapAngle = null;
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
