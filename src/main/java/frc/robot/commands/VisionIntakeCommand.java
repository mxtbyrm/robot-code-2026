package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import org.littletonrobotics.junction.Logger;

import frc.robot.constants.AutoConstants;
import frc.robot.constants.VisionConstants;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.Vision;

/**
 * Vision-assisted ball chase command.
 *
 * <p>Uses the intake camera's object detection to drive toward a detected ball.
 * Steers left/right based on ball yaw, drives forward based on ball area.
 * Meant to run in parallel with intake commands.
 *
 * <h2>Modes:</h2>
 * <ul>
 *   <li><b>Teleop</b> (default constructor): never auto-finishes — runs until button released</li>
 *   <li><b>Auto</b> ({@link #forAuto}): finishes when ball is close enough or timeout expires</li>
 * </ul>
 */
public class VisionIntakeCommand extends Command {

    private final SwerveDrive swerveDrive;
    private final Vision vision;

    /** If true, isFinished() returns true when ball is close enough. */
    private final boolean autoMode;

    // PID for steering toward the ball (yaw correction)
    private final PIDController yawController = new PIDController(
            VisionConstants.kBallChaseYawP, VisionConstants.kBallChaseYawI, VisionConstants.kBallChaseYawD);

    // When ball area exceeds this, we're close enough → stop
    private static final double kCloseEnoughArea = VisionConstants.kBallCloseEnoughArea;

    // Forward speed scales inversely with area (closer = slower)
    private static final double kMaxChaseSpeed = VisionConstants.kBallChaseMaxSpeed;
    private static final double kMinChaseSpeed = VisionConstants.kBallChaseMinSpeed;

    // Debounce: wait before spinning to search after losing a ball
    private static final double kBallLostDebounceSeconds = VisionConstants.kBallLostDebounceSeconds;
    private double lastBallSeenTimestamp = 0.0;

    /**
     * Teleop constructor — command never auto-finishes.
     * Bind with {@code whileTrue()} so button release ends the chase.
     */
    public VisionIntakeCommand(SwerveDrive swerveDrive, Vision vision) {
        this(swerveDrive, vision, false);
    }

    /**
     * Internal constructor with auto-mode flag.
     *
     * @param autoMode if true, {@link #isFinished()} returns true when ball area ≥ threshold
     */
    private VisionIntakeCommand(SwerveDrive swerveDrive, Vision vision, boolean autoMode) {
        this.swerveDrive = swerveDrive;
        this.vision = vision;
        this.autoMode = autoMode;

        yawController.setSetpoint(0); // target: ball centered in frame
        yawController.setTolerance(VisionConstants.kBallChaseYawTolerance);

        addRequirements(swerveDrive);
    }

    /**
     * Create a vision chase command for autonomous use.
     *
     * <p>Unlike the teleop version, this command:
     * <ul>
     *   <li>Finishes when the ball is close enough (area ≥ threshold)</li>
     *   <li>Has a timeout to prevent wasting auto time if no ball is visible</li>
     * </ul>
     *
     * <p>Designed to be used with {@code Commands.deadline()} alongside intake commands.
     *
     * @param swerveDrive the swerve drive subsystem
     * @param vision the vision subsystem
     * @return a timeout-wrapped vision chase command
     */
    public static Command forAuto(SwerveDrive swerveDrive, Vision vision) {
        return new VisionIntakeCommand(swerveDrive, vision, true)
                .withTimeout(AutoConstants.kVisionChaseTimeoutSeconds)
                .withName("VisionIntake: Auto Chase");
    }

    @Override
    public void initialize() {
        // Set to current time so we don't immediately spin-search on start
        lastBallSeenTimestamp = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        if (!vision.isBallDetected()) {
            // Ball lost — wait before spinning to search (prevents jitter on momentary loss)
            double timeSinceSeen = Timer.getFPGATimestamp() - lastBallSeenTimestamp;
            if (timeSinceSeen < kBallLostDebounceSeconds) {
                // Recently saw the ball — hold position, don't spin yet
                swerveDrive.drive(0, 0, 0, false);
            } else {
                // Ball lost for a while — spin slowly to search
                swerveDrive.drive(0, 0, 1.0, false);
            }
            return;
        }

        // Ball detected — update last seen timestamp
        lastBallSeenTimestamp = Timer.getFPGATimestamp();

        double yaw = vision.getBallYaw();
        double area = vision.getBallArea();

        // Steer toward ball
        double rotSpeed = yawController.calculate(-yaw); // negative because yaw positive = ball left
        rotSpeed = MathUtil.clamp(rotSpeed, -2.0, 2.0);

        // Drive forward — speed inversely proportional to area (closer = slower)
        double forwardSpeed = MathUtil.interpolate(kMaxChaseSpeed, kMinChaseSpeed,
                MathUtil.clamp(area / kCloseEnoughArea, 0, 1));

        swerveDrive.drive(forwardSpeed, 0, rotSpeed, false);

        Logger.recordOutput("VisionIntake/Yaw", yaw);
        Logger.recordOutput("VisionIntake/Area", area);
        Logger.recordOutput("VisionIntake/ForwardSpeed", forwardSpeed);
    }

    @Override
    public void end(boolean interrupted) {
        swerveDrive.stop();
    }

    @Override
    public boolean isFinished() {
        if (autoMode && vision.isBallDetected()) {
            // In auto mode, finish when ball is close enough to be collected
            return vision.getBallArea() >= kCloseEnoughArea;
        }
        // Teleop: never auto-finish — let the button release end this command
        return false;
    }
}
