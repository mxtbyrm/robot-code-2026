package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.commands.PathfindingCommand;

import choreo.trajectory.SwerveSample;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.constants.SwerveConstants;
import frc.robot.sim.SwerveDriveSim;
import frc.robot.RobotState;

import org.littletonrobotics.junction.Logger;

/**
 * Swerve drivetrain subsystem using 4 MK4i modules with Kraken X60 motors and CANcoders.
 * 
 * Features:
 * - Field-relative and robot-relative driving
 * - SwerveDrivePoseEstimator for odometry (auto-compatible, can fuse vision)
 * - Full telemetry via AdvantageKit Logger and NetworkTables
 * - Ready for autonomous integration (ChassisSpeeds-based interface)
 */
public class SwerveDrive extends SubsystemBase {

    // ==================== SINGLETON ====================
    private static SwerveDrive instance;

    public static void initialize() {
        if (instance != null) throw new IllegalStateException("SwerveDrive already initialized.");
        instance = new SwerveDrive();
    }

    public static SwerveDrive getInstance() {
        if (instance == null) throw new IllegalStateException("SwerveDrive not initialized. Call initialize() first.");
        return instance;
    }

    public static void resetInstance() { instance = null; }

    // ==================== MODULES ====================
    private final SwerveModule frontLeft;
    private final SwerveModule frontRight;
    private final SwerveModule backLeft;
    private final SwerveModule backRight;

    private final SwerveModule[] modules;

    // ==================== SENSORS ====================
    private final Pigeon2 pigeon;

    // ==================== ODOMETRY ====================
    private final SwerveDrivePoseEstimator poseEstimator;

    // ==================== TELEMETRY ====================
    private final Field2d field2d = new Field2d();
    private final StructArrayPublisher<SwerveModuleState> statePublisher;
    private final StructArrayPublisher<SwerveModuleState> desiredStatePublisher;
    private final StructPublisher<Pose2d> posePublisher;

    // ==================== SLIP DETECTION ====================
    /** Last commanded module speeds (m/s). Compared against actual encoder velocity. */
    private final double[] lastCommandedSpeeds = new double[4];

    // ==================== PRE-ALLOCATED ARRAYS (zero GC in steady state) ====================
    private final SwerveModuleState[] cachedModuleStates = {
            new SwerveModuleState(), new SwerveModuleState(),
            new SwerveModuleState(), new SwerveModuleState()
    };
    private final SwerveModulePosition[] cachedModulePositions = {
            new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()
    };
    // Separate array for the 250Hz odometry thread to avoid cross-thread mutation
    private final SwerveModulePosition[] odometryPositions = {
            new SwerveModulePosition(), new SwerveModulePosition(),
            new SwerveModulePosition(), new SwerveModulePosition()
    };

    // ==================== SIMULATION ====================
    private SwerveDriveSim swerveSim = null;

    // ==================== 250Hz ODOMETRY THREAD ====================
    /**
     * High-frequency odometry thread running at 250Hz (4ms period).
     * Updates the pose estimator between main loop cycles for smoother,
     * more accurate position tracking — especially important during
     * fast driving and shoot-while-moving.
     */
    private final Notifier odometryNotifier;
    private final Object odometryLock = new Object();

    private SwerveDrive() {
        // Initialize Pigeon2 IMU
        pigeon = new Pigeon2(SwerveConstants.kPigeonId, SwerveConstants.kCANivoreBus);
        pigeon.reset();
        pigeon.getYaw().setUpdateFrequency(SwerveConstants.kPigeonYawUpdateFreqHz);
        pigeon.getAngularVelocityZWorld().setUpdateFrequency(SwerveConstants.kPigeonAngVelUpdateFreqHz);
        pigeon.optimizeBusUtilization();

        // Initialize swerve modules
        // MK4i with all bevel gears facing left:
        // Left side (FL, BL) drive motors are inverted
        // Right side (FR, BR) drive motors are NOT inverted
        frontLeft = new SwerveModule(
                "FL",
                SwerveConstants.kFrontLeftDriveMotorId,
                SwerveConstants.kFrontLeftSteerMotorId,
                SwerveConstants.kFrontLeftEncoderId,
                SwerveConstants.kFrontLeftEncoderOffset,
                true // drive inverted (bevel gear side)
        );

        frontRight = new SwerveModule(
                "FR",
                SwerveConstants.kFrontRightDriveMotorId,
                SwerveConstants.kFrontRightSteerMotorId,
                SwerveConstants.kFrontRightEncoderId,
                SwerveConstants.kFrontRightEncoderOffset,
                false // drive not inverted
        );

        backLeft = new SwerveModule(
                "BL",
                SwerveConstants.kBackLeftDriveMotorId,
                SwerveConstants.kBackLeftSteerMotorId,
                SwerveConstants.kBackLeftEncoderId,
                SwerveConstants.kBackLeftEncoderOffset,
                true // drive inverted (bevel gear side)
        );

        backRight = new SwerveModule(
                "BR",
                SwerveConstants.kBackRightDriveMotorId,
                SwerveConstants.kBackRightSteerMotorId,
                SwerveConstants.kBackRightEncoderId,
                SwerveConstants.kBackRightEncoderOffset,
                false // drive not inverted
        );

        modules = new SwerveModule[] { frontLeft, frontRight, backLeft, backRight };

        // Initialize pose estimator with starting position at origin
        poseEstimator = new SwerveDrivePoseEstimator(
                SwerveConstants.kSwerveKinematics,
                getGyroYaw(),
                getModulePositions(),
                new Pose2d()
        );

        // NetworkTables publishers for AdvantageScope / Elastic / Glass visualization
        statePublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Swerve/ModuleStates", SwerveModuleState.struct)
                .publish();
        desiredStatePublisher = NetworkTableInstance.getDefault()
                .getStructArrayTopic("Swerve/DesiredStates", SwerveModuleState.struct)
                .publish();
        posePublisher = NetworkTableInstance.getDefault()
                .getStructTopic("Swerve/Pose", Pose2d.struct)
                .publish();

        SmartDashboard.putData("Field", field2d);

        // Initialize sim physics if running in simulation
        if (RobotBase.isSimulation()) {
            swerveSim = new SwerveDriveSim(modules, pigeon);
        }

        // Start 250Hz odometry thread for high-frequency pose updates
        odometryNotifier = new Notifier(this::updateOdometryHighFreq);
        odometryNotifier.setName("OdometryThread");
        odometryNotifier.startPeriodic(SwerveConstants.kOdometryPeriodSeconds);

        // ==================== PATHPLANNER AUTOBUILDER CONFIGURATION ====================
        // Configure AutoBuilder for on-the-fly pathfinding and path following.
        // This enables AutoBuilder.pathfindToPose() and AutoBuilder.followPath() commands.
        try {
            RobotConfig config = RobotConfig.fromGUISettings();

            AutoBuilder.configure(
                    this::getPose,
                    this::resetPose,
                    this::getRobotRelativeSpeeds,
                    this::driveFromChassisSpeeds,
                    new PPHolonomicDriveController(
                            new PIDConstants(
                                    SwerveConstants.kAutoTranslationP,
                                    SwerveConstants.kAutoTranslationI,
                                    SwerveConstants.kAutoTranslationD),
                            new PIDConstants(
                                    SwerveConstants.kAutoRotationP,
                                    SwerveConstants.kAutoRotationI,
                                    SwerveConstants.kAutoRotationD)
                    ),
                    config,
                    // Flip paths for red alliance (blue origin coordinate system)
                    () -> {
                        var alliance = DriverStation.getAlliance();
                        return alliance.isPresent()
                                && alliance.get() == DriverStation.Alliance.Red;
                    },
                    this // SwerveDrive is the drive subsystem requirement
            );
        } catch (Exception e) {
            DriverStation.reportError(
                    "[SwerveDrive] Failed to configure AutoBuilder: " + e.getMessage(), e.getStackTrace());
        }

        // Warm up PathPlannerLib to avoid cold-start delay on first pathfinding command
        CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());
    }

    /**
     * High-frequency odometry update (runs at 250Hz in a separate thread).
     * Only updates the pose estimator — no telemetry, no health checks.
     * Skips the update if any module's CAN signals failed to refresh.
     */
    private void updateOdometryHighFreq() {
        boolean allSignalsOk = true;
        for (SwerveModule module : modules) {
            if (!module.refreshSignalsChecked()) {
                allSignalsOk = false;
            }
        }

        if (allSignalsOk) {
            for (int i = 0; i < 4; i++) {
                odometryPositions[i].distanceMeters = modules[i].getDrivePosition();
                odometryPositions[i].angle = modules[i].getAngle();
            }
            synchronized (odometryLock) {
                poseEstimator.update(getGyroYaw(), odometryPositions);
            }
        }
    }

    // ==================== PERIODIC ====================

    @Override
    public void periodic() {
        // Signal refresh and odometry update are handled by the 250Hz background thread.
        // The main thread only reads the cached (volatile) values for telemetry and health.
        // No need to call poseEstimator.update() here — the 250Hz thread already does it.

        // ---- Signal health + module health ----
        boolean allModulesOk = true;
        for (SwerveModule module : modules) {
            module.checkSignalHealth();
            if (!module.isHealthy()) {
                allModulesOk = false;
                DriverStation.reportWarning(
                        "[SwerveDrive] Module " + module.getName() + " unhealthy!", false);
            }
        }
        RobotState.getInstance().setSwerveHealthy(allModulesOk);

        // ---- Slip detection ----
        for (int i = 0; i < 4; i++) {
            double commanded = lastCommandedSpeeds[i];
            double actual = Math.abs(modules[i].getDriveVelocity());
            boolean slipping = commanded > SwerveConstants.kSlipMinCommandedSpeed
                    && actual > SwerveConstants.kSlipMinActualSpeed
                    && Math.abs(commanded - actual) / commanded > SwerveConstants.kSlipDetectionThreshold;
            Logger.recordOutput("Swerve/" + modules[i].getName() + "Slipping", slipping);
        }

        // ---- Gyro drift detection ----
        ChassisSpeeds kinematicSpeeds = getRobotRelativeSpeeds();
        double kinematicOmega = kinematicSpeeds.omegaRadiansPerSecond;
        double gyroOmega = Math.toRadians(pigeon.getAngularVelocityZWorld().getValueAsDouble());
        double omegaDiscrepancy = Math.abs(gyroOmega - kinematicOmega);
        Logger.recordOutput("Swerve/GyroDriftDiscrepancy", omegaDiscrepancy);
        Logger.recordOutput("Swerve/GyroDriftWarning", omegaDiscrepancy > 0.5);

        // Publish telemetry
        Pose2d pose = getPose();
        SwerveModuleState[] currentStates = getModuleStates();
        field2d.setRobotPose(pose);
        posePublisher.set(pose);
        statePublisher.set(currentStates);

        // AdvantageKit structured logging
        Logger.recordOutput("Swerve/Pose", pose);
        Logger.recordOutput("RobotState/Pose3d", new edu.wpi.first.math.geometry.Pose3d(pose));
        Logger.recordOutput("Swerve/ModuleStates", currentStates);
        Logger.recordOutput("Swerve/GyroHeading", getHeading().getDegrees());
        Logger.recordOutput("Swerve/AllModulesHealthy", allModulesOk);

        // ---- Module 3D poses for AdvantageScope 3D swerve visualization ----
        // Each module's position in field frame + steering angle as yaw.
        double cosH = Math.cos(pose.getRotation().getRadians());
        double sinH = Math.sin(pose.getRotation().getRadians());
        double halfBase  = SwerveConstants.kWheelBaseMeters  / 2.0;
        double halfTrack = SwerveConstants.kTrackWidthMeters / 2.0;
        // Robot-frame offsets: FL, FR, BL, BR (matches modules[] order)
        double[] robotModX = { halfBase,  halfBase,  -halfBase, -halfBase };
        double[] robotModY = { halfTrack, -halfTrack, halfTrack, -halfTrack };
        edu.wpi.first.math.geometry.Pose3d[] modulePoses3d =
                new edu.wpi.first.math.geometry.Pose3d[4];
        for (int i = 0; i < 4; i++) {
            double fieldX = pose.getX() + robotModX[i] * cosH - robotModY[i] * sinH;
            double fieldY = pose.getY() + robotModX[i] * sinH + robotModY[i] * cosH;
            double moduleYaw = pose.getRotation().getRadians()
                    + currentStates[i].angle.getRadians();
            modulePoses3d[i] = new edu.wpi.first.math.geometry.Pose3d(
                    fieldX, fieldY, SwerveConstants.kWheelDiameterMeters / 2.0,
                    new edu.wpi.first.math.geometry.Rotation3d(0, 0, moduleYaw));
        }
        Logger.recordOutput("Swerve/ModulePoses3d", modulePoses3d);
    }

    // ==================== SIMULATION ====================

    @Override
    public void simulationPeriodic() {
        if (swerveSim != null) {
            swerveSim.update(SwerveConstants.kMainLoopPeriodSeconds);
        }
    }

    // ==================== DRIVING ====================

    /**
     * Drive field-relative using joystick inputs.
     *
     * @param xSpeed        Speed in the X direction (forward positive), m/s
     * @param ySpeed        Speed in the Y direction (left positive), m/s
     * @param rotSpeed      Angular speed (CCW positive), rad/s
     * @param fieldRelative Whether to use field-relative control
     */
    public void drive(double xSpeed, double ySpeed, double rotSpeed, boolean fieldRelative) {
        ChassisSpeeds speeds;
        if (fieldRelative) {
            speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed, getHeading());
        } else {
            speeds = new ChassisSpeeds(xSpeed, ySpeed, rotSpeed);
        }

        driveFromChassisSpeeds(speeds);
    }

    /**
     * Drive using ChassisSpeeds. This is the primary interface used by autonomous controllers.
     *
     * @param speeds The desired chassis speeds
     */
    public void driveFromChassisSpeeds(ChassisSpeeds speeds) {
        // Brownout protection: scale speeds when battery is low
        double voltComp = RobotState.getInstance().getVoltageCompensationFactor();
        if (voltComp < 1.0) {
            speeds = new ChassisSpeeds(
                    speeds.vxMetersPerSecond * voltComp,
                    speeds.vyMetersPerSecond * voltComp,
                    speeds.omegaRadiansPerSecond * voltComp);
        }

        // Discretize to reduce drift when translating and rotating simultaneously
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, SwerveConstants.kMainLoopPeriodSeconds);

        // Second-order kinematics: predict module states at the midpoint of the
        // control interval (trapezoidal approximation). This accounts for acceleration
        // and reduces the systematic lag between commanded and achieved trajectory.
        ChassisSpeeds currentSpeeds = getRobotRelativeSpeeds();
        ChassisSpeeds predictedSpeeds = new ChassisSpeeds(
                discreteSpeeds.vxMetersPerSecond
                        + (discreteSpeeds.vxMetersPerSecond - currentSpeeds.vxMetersPerSecond) * 0.5,
                discreteSpeeds.vyMetersPerSecond
                        + (discreteSpeeds.vyMetersPerSecond - currentSpeeds.vyMetersPerSecond) * 0.5,
                discreteSpeeds.omegaRadiansPerSecond
                        + (discreteSpeeds.omegaRadiansPerSecond - currentSpeeds.omegaRadiansPerSecond) * 0.5);

        SwerveModuleState[] states = SwerveConstants.kSwerveKinematics.toSwerveModuleStates(predictedSpeeds);

        // Normalize wheel speeds if any exceed the max
        SwerveDriveKinematics.desaturateWheelSpeeds(states, SwerveConstants.kMaxSpeedMetersPerSecond);

        setModuleStates(states);

        // Publish desired states for telemetry
        desiredStatePublisher.set(states);
    }

    /**
     * Sets individual module states. Used by driveFromChassisSpeeds and auto paths.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        for (int i = 0; i < 4; i++) {
            modules[i].setDesiredState(desiredStates[i]);
            lastCommandedSpeeds[i] = Math.abs(desiredStates[i].speedMetersPerSecond);
        }
    }

    /**
     * Stops all swerve modules.
     */
    public void stop() {
        for (SwerveModule module : modules) {
            module.stop();
        }
    }

    /**
     * Sets wheels to X-pattern for locking in place.
     */
    public void lockWheels() {
        frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
        backRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    }

    // ==================== ODOMETRY / POSE ====================

    /**
     * Gets the current estimated pose from the pose estimator.
     */
    public Pose2d getPose() {
        synchronized (odometryLock) {
            return poseEstimator.getEstimatedPosition();
        }
    }

    /**
     * Resets the pose estimator to a specific pose.
     * Used at the start of autonomous to set the starting position.
     *
     * @param pose The pose to reset to
     */
    public void resetPose(Pose2d pose) {
        synchronized (odometryLock) {
            poseEstimator.resetPosition(getGyroYaw(), getModulePositions(), pose);
        }
    }

    /**
     * Adds a vision measurement to the pose estimator for sensor fusion.
     * Called by the Vision subsystem each time a valid AprilTag pose is estimated.
     *
     * @param visionPose      The estimated robot pose from vision
     * @param timestampSeconds The timestamp of the vision measurement (from PhotonVision)
     * @param stdDevs          The standard deviations for (x, y, heading) of this measurement
     */
    public void addVisionMeasurement(Pose2d visionPose, double timestampSeconds, Matrix<N3, N1> stdDevs) {
        double age = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - timestampSeconds;
        if (age > SwerveConstants.kMaxVisionMeasurementAgeSec) {
            Logger.recordOutput("Swerve/VisionRejectedAge", age);
            return;
        }

        synchronized (odometryLock) {
            poseEstimator.addVisionMeasurement(visionPose, timestampSeconds, stdDevs);
        }
    }

    /**
     * Gets the current robot-relative chassis speeds.
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return SwerveConstants.kSwerveKinematics.toChassisSpeeds(getModuleStates());
    }

    /**
     * Gets the current field-relative chassis speeds.
     * Used by the shooter for shoot-while-moving compensation.
     */
    public ChassisSpeeds getFieldRelativeSpeeds() {
        ChassisSpeeds robotRelative = getRobotRelativeSpeeds();
        return ChassisSpeeds.fromRobotRelativeSpeeds(robotRelative, getHeading());
    }

    // ==================== GYRO ====================

    /**
     * Gets the Pigeon2 yaw as a Rotation2d.
     */
    public Rotation2d getGyroYaw() {
        return pigeon.getRotation2d();
    }

    /**
     * Gets the robot heading from the pose estimator (fused heading).
     */
    public Rotation2d getHeading() {
        return getPose().getRotation();
    }

    /**
     * Zeros the gyroscope heading. The robot will consider its current orientation as "forward".
     */
    public void zeroGyro() {
        pigeon.reset();
    }

    // ==================== MODULE STATE HELPERS ====================

    /**
     * Gets the current positions of all modules (pre-allocated, main thread only).
     */
    public SwerveModulePosition[] getModulePositions() {
        for (int i = 0; i < 4; i++) {
            cachedModulePositions[i].distanceMeters = modules[i].getDrivePosition();
            cachedModulePositions[i].angle = modules[i].getAngle();
        }
        return cachedModulePositions;
    }

    /**
     * Gets the current states of all modules (pre-allocated).
     */
    public SwerveModuleState[] getModuleStates() {
        for (int i = 0; i < 4; i++) {
            cachedModuleStates[i].speedMetersPerSecond = modules[i].getDriveVelocity();
            cachedModuleStates[i].angle = modules[i].getAngle();
        }
        return cachedModuleStates;
    }

    /**
     * Gets the swerve drive kinematics (needed by PathPlanner / Choreo).
     */
    public SwerveDriveKinematics getKinematics() {
        return SwerveConstants.kSwerveKinematics;
    }

    // ==================== CHOREO TRAJECTORY FOLLOWING ====================

    private final PIDController autoXController = new PIDController(
            SwerveConstants.kAutoTranslationP,
            SwerveConstants.kAutoTranslationI,
            SwerveConstants.kAutoTranslationD);
    private final PIDController autoYController = new PIDController(
            SwerveConstants.kAutoTranslationP,
            SwerveConstants.kAutoTranslationI,
            SwerveConstants.kAutoTranslationD);
    private final PIDController autoHeadingController = new PIDController(
            SwerveConstants.kAutoRotationP,
            SwerveConstants.kAutoRotationI,
            SwerveConstants.kAutoRotationD);

    {
        // Enable continuous input for heading so it wraps around -PI to PI
        autoHeadingController.enableContinuousInput(-Math.PI, Math.PI);
    }

    /**
     * Follows a Choreo SwerveSample by applying feedforward velocities
     * with PID feedback corrections. This is called by ChoreoLib's AutoFactory.
     *
     * @param sample The trajectory sample to follow
     */
    public void followTrajectory(SwerveSample sample) {
        Pose2d pose = getPose();

        ChassisSpeeds speeds = new ChassisSpeeds(
                sample.vx + autoXController.calculate(pose.getX(), sample.x),
                sample.vy + autoYController.calculate(pose.getY(), sample.y),
                sample.omega + autoHeadingController.calculate(
                        pose.getRotation().getRadians(), sample.heading)
        );

        driveFieldRelative(speeds);
    }

    /**
     * Drives the robot using field-relative ChassisSpeeds.
     * Used by the trajectory follower (speeds are already field-relative).
     *
     * @param fieldRelativeSpeeds Field-relative chassis speeds
     */
    public void driveFieldRelative(ChassisSpeeds fieldRelativeSpeeds) {
        ChassisSpeeds robotRelative = ChassisSpeeds.fromFieldRelativeSpeeds(
                fieldRelativeSpeeds, getHeading());
        driveFromChassisSpeeds(robotRelative);
    }

    // ==================== SYSID CHARACTERIZATION ====================

    /**
     * Applies a raw voltage to all drive motors with modules pointed forward.
     * Used by SysId to characterize drive motor feedforward (kS, kV, kA).
     */
    public void runDriveCharacterization(double volts) {
        for (SwerveModule module : modules) {
            module.setDesiredState(new SwerveModuleState(0, new Rotation2d()));
        }
        for (SwerveModule module : modules) {
            module.setDriveVoltage(volts);
        }
    }

    /**
     * Applies a raw voltage to all steer motors simultaneously.
     * Used by SysId to characterize steer motor feedforward (kS) and PID gains.
     * Drive motors are stopped during steer characterization.
     */
    public void runSteerCharacterization(double volts) {
        for (SwerveModule module : modules) {
            module.setDriveVoltage(0);
            module.setSteerVoltage(volts);
        }
    }

    /** Returns the modules array for SysId logging. */
    public SwerveModule[] getModules() {
        return modules;
    }
}
