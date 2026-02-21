package frc.robot.subsystems;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DriverStation;

import frc.robot.constants.SwerveConstants;

/**
 * Einstein-grade MK4i swerve module.
 *
 * <h2>Improvements over basic implementation:</h2>
 * <ul>
 *   <li><b>Cosine compensation</b> — scales drive speed by cos(angle_error) during steering transitions</li>
 *   <li><b>Optimize against last commanded</b> — prevents 180° flips at low speed from encoder noise</li>
 *   <li><b>Anti-jitter dead zone</b> — holds last steer angle when drive speed is below threshold</li>
 *   <li><b>Config retry loop</b> — retries config apply up to 5× on CAN failure</li>
 *   <li><b>Latency-compensated odometry</b> — uses Phoenix6 synchronized timestamps</li>
 *   <li><b>Stator current limiting</b> — prevents wheel slip + brownout</li>
 *   <li><b>Voltage compensation</b> — consistent behavior across battery levels</li>
 *   <li><b>Signal health monitoring</b> — detects stale CAN signals at runtime</li>
 *   <li><b>Bus optimization</b> — unused signals disabled to free CAN bandwidth</li>
 * </ul>
 */
public class SwerveModule {

    // ==================== HARDWARE ====================
    private final TalonFX driveMotor;
    private final TalonFX steerMotor;
    private final CANcoder cancoder;

    // ==================== CONTROL REQUESTS (reused — zero GC) ====================
    private final VelocityVoltage driveRequest = new VelocityVoltage(0).withSlot(0);
    private final PositionVoltage steerRequest = new PositionVoltage(0).withSlot(0);

    // ==================== STATUS SIGNALS ====================
    private final StatusSignal<Angle> drivePositionSignal;
    private final StatusSignal<AngularVelocity> driveVelocitySignal;
    private final StatusSignal<Angle> steerPositionSignal;
    private final StatusSignal<AngularVelocity> steerVelocitySignal;

    // Latency-compensated cache — updated atomically in refreshSignals()
    // Volatile for cross-thread visibility (250Hz odometry thread + main thread)
    private volatile double cachedDrivePositionRot = 0;
    private volatile double cachedSteerPositionRot = 0;

    /** Last meaningful steer angle — held when speed is near zero to prevent snap-to-0°. */
    private Rotation2d lastSteerAngle = new Rotation2d();

    private final String moduleName;

    // ==================== FAULT TRACKING ====================
    private boolean configHealthy = true;
    private boolean signalHealthy = true;
    private double lastSignalTimestamp = 0.0;
    private static final int kMaxConfigRetries = 5;

    public SwerveModule(
            String moduleName,
            int driveMotorId,
            int steerMotorId,
            int cancoderId,
            double cancoderOffset,
            boolean driveInverted) {

        this.moduleName = moduleName;

        driveMotor = new TalonFX(driveMotorId, SwerveConstants.kCANivoreBus);
        steerMotor = new TalonFX(steerMotorId, SwerveConstants.kCANivoreBus);
        cancoder = new CANcoder(cancoderId, SwerveConstants.kCANivoreBus);

        // ==================== CANcoder Configuration ====================
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = cancoderOffset;
        cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        applyConfig(() -> cancoder.getConfigurator().apply(cancoderConfig), "CANcoder");

        // ==================== Drive Motor Configuration ====================
        TalonFXConfiguration driveConfig = new TalonFXConfiguration();

        driveConfig.Slot0.kP = SwerveConstants.kDriveP;
        driveConfig.Slot0.kI = SwerveConstants.kDriveI;
        driveConfig.Slot0.kD = SwerveConstants.kDriveD;
        driveConfig.Slot0.kS = SwerveConstants.kDriveS;
        driveConfig.Slot0.kV = SwerveConstants.kDriveV;
        driveConfig.Slot0.kA = SwerveConstants.kDriveA;

        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.kDriveCurrentLimit;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = SwerveConstants.kDriveStatorCurrentLimit;

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveConfig.MotorOutput.Inverted = driveInverted
                ? InvertedValue.Clockwise_Positive
                : InvertedValue.CounterClockwise_Positive;

        driveConfig.Feedback.SensorToMechanismRatio = SwerveConstants.kDriveGearRatio;

        driveConfig.Voltage.PeakForwardVoltage = SwerveConstants.kPeakForwardVoltage;
        driveConfig.Voltage.PeakReverseVoltage = SwerveConstants.kPeakReverseVoltage;

        applyConfig(() -> driveMotor.getConfigurator().apply(driveConfig), "Drive");

        // ==================== Steer Motor Configuration ====================
        TalonFXConfiguration steerConfig = new TalonFXConfiguration();

        steerConfig.Slot0.kP = SwerveConstants.kSteerP;
        steerConfig.Slot0.kI = SwerveConstants.kSteerI;
        steerConfig.Slot0.kD = SwerveConstants.kSteerD;
        steerConfig.Slot0.kS = SwerveConstants.kSteerS;

        steerConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        steerConfig.CurrentLimits.SupplyCurrentLimit = SwerveConstants.kSteerCurrentLimit;

        steerConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        steerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        steerConfig.Feedback.FeedbackRemoteSensorID = cancoderId;
        steerConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        steerConfig.Feedback.RotorToSensorRatio = SwerveConstants.kSteerGearRatio;
        steerConfig.Feedback.SensorToMechanismRatio = 1.0;

        steerConfig.ClosedLoopGeneral.ContinuousWrap = true;

        steerConfig.Voltage.PeakForwardVoltage = SwerveConstants.kPeakForwardVoltage;
        steerConfig.Voltage.PeakReverseVoltage = SwerveConstants.kPeakReverseVoltage;

        applyConfig(() -> steerMotor.getConfigurator().apply(steerConfig), "Steer");

        // ==================== Cache Status Signals ====================
        drivePositionSignal = driveMotor.getPosition();
        driveVelocitySignal = driveMotor.getVelocity();
        steerPositionSignal = steerMotor.getPosition();
        steerVelocitySignal = steerMotor.getVelocity();

        BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.kModulePositionUpdateFreqHz,
                drivePositionSignal, steerPositionSignal);
        BaseStatusSignal.setUpdateFrequencyForAll(SwerveConstants.kModuleVelocityUpdateFreqHz,
                driveVelocitySignal, steerVelocitySignal);
        // cancoderPositionSignal is NOT subscribed here — FusedCANcoder feedback
        // is handled internally by the TalonFX. No need to read it from user code.

        driveMotor.optimizeBusUtilization();
        steerMotor.optimizeBusUtilization();
        cancoder.optimizeBusUtilization();
    }

    /** Retry loop for config — handles transient CAN failures at startup. */
    private void applyConfig(java.util.function.Supplier<StatusCode> apply, String device) {
        StatusCode status = StatusCode.StatusCodeNotInitialized;
        for (int i = 0; i < kMaxConfigRetries; i++) {
            status = apply.get();
            if (status.isOK()) return;
        }
        configHealthy = false;
        DriverStation.reportError(
                "[" + moduleName + "] " + device + " config FAILED: " + status, false);
    }

    // ==================== DRIVING ====================

    public void setDesiredState(SwerveModuleState desiredState) {
        // Optimize against LAST COMMANDED angle, not measured encoder angle.
        // At low speeds, encoder noise can cause the measured angle to jitter
        // across the 90° optimization boundary, triggering false 180° reversals.
        // Using the last commanded angle is deterministic and avoids this.
        desiredState.optimize(lastSteerAngle);

        // Anti-jitter: when speed is below threshold, hold last angle.
        // Kinematics produces a meaningless 0° angle at zero speed.
        if (Math.abs(desiredState.speedMetersPerSecond) > SwerveConstants.kAntiJitterSpeedMPS) {
            lastSteerAngle = desiredState.angle;
        }

        // Cosine compensation: while the module is still rotating toward the
        // target angle, driving at full speed scrubs sideways (the wheel isn't
        // pointed where kinematics expects). Scale drive speed by cos(error)
        // so the effective forward component matches what was requested.
        double angleErrorRad = desiredState.angle.minus(getAngle()).getRadians();
        double cosineScalar = Math.cos(angleErrorRad);
        double adjustedSpeed = desiredState.speedMetersPerSecond * cosineScalar;

        double driveRPS = adjustedSpeed / SwerveConstants.kWheelCircumferenceMeters;
        driveMotor.setControl(driveRequest.withVelocity(driveRPS));
        steerMotor.setControl(steerRequest.withPosition(lastSteerAngle.getRotations()));
    }

    // ==================== SIGNAL READING (latency-compensated) ====================

    /** Gets angle from the latency-compensated cache (thread-safe). */
    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(cachedSteerPositionRot);
    }

    public double getDriveVelocity() {
        return driveVelocitySignal.getValueAsDouble() * SwerveConstants.kWheelCircumferenceMeters;
    }

    /** Gets drive position in meters (latency-compensated). */
    public double getDrivePosition() {
        return cachedDrivePositionRot * SwerveConstants.kWheelCircumferenceMeters;
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), getAngle());
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getAngle());
    }

    /**
     * Atomically refreshes all signals + applies latency compensation.
     * Call ONCE per loop BEFORE reading positions.
     */
    public void refreshSignals() {
        refreshSignalsChecked();
    }

    /**
     * Refreshes signals and returns whether all refreshed successfully.
     * Used by the odometry thread to skip pose updates on CAN failure.
     */
    public boolean refreshSignalsChecked() {
        StatusCode status = BaseStatusSignal.refreshAll(
                drivePositionSignal, driveVelocitySignal,
                steerPositionSignal, steerVelocitySignal);

        if (!status.isOK()) {
            return false;
        }

        // Latency compensation: position += velocity × CAN transit latency
        cachedDrivePositionRot = BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                drivePositionSignal, driveVelocitySignal);
        cachedSteerPositionRot = BaseStatusSignal.getLatencyCompensatedValueAsDouble(
                steerPositionSignal, steerVelocitySignal);
        return true;
    }

    /**
     * Checks if CAN signals are fresh. Call periodically from SwerveDrive.periodic().
     * If a signal hasn't updated beyond the age threshold, the module is marked unhealthy.
     */
    public void checkSignalHealth() {
        double driveTimestamp = drivePositionSignal.getTimestamp().getTime();
        if (driveTimestamp == lastSignalTimestamp) {
            double age = edu.wpi.first.wpilibj.Timer.getFPGATimestamp() - driveTimestamp;
            if (age > SwerveConstants.kMaxSignalAgeSeconds) {
                signalHealthy = false;
            }
        } else {
            lastSignalTimestamp = driveTimestamp;
            signalHealthy = true;
        }
    }

    /** Returns signals for odometry thread registration. */
    public BaseStatusSignal[] getOdometrySignals() {
        return new BaseStatusSignal[] {
                drivePositionSignal, driveVelocitySignal,
                steerPositionSignal, steerVelocitySignal
        };
    }

    public void stop() {
        driveMotor.stopMotor();
        steerMotor.stopMotor();
    }

    /** Apply raw voltage to the drive motor (for SysId characterization). */
    public void setDriveVoltage(double volts) {
        driveMotor.setVoltage(volts);
    }

    /** Gets the drive motor velocity in wheel-shaft RPS (for SysId logging). */
    public double getDriveVelocityRPS() {
        return driveVelocitySignal.getValueAsDouble();
    }

    /** Gets the drive motor position in wheel-shaft rotations (for SysId logging). */
    public double getDrivePositionRotations() {
        return cachedDrivePositionRot;
    }

    /** Apply raw voltage to the steer motor (for SysId characterization). */
    public void setSteerVoltage(double volts) {
        steerMotor.setVoltage(volts);
    }

    /** Gets the steer motor velocity in azimuth RPS (mechanism, 1 = full wheel rotation). */
    public double getSteerVelocityRPS() {
        return steerVelocitySignal.getValueAsDouble();
    }

    /** Gets the steer motor position in azimuth rotations (mechanism, 0–1 per full rotation). */
    public double getSteerPositionRotations() {
        return cachedSteerPositionRot;
    }

    // ==================== HEALTH ====================

    /** Whether config succeeded at boot AND CAN signals are fresh. */
    public boolean isHealthy() {
        return configHealthy && signalHealthy;
    }

    public String getName() {
        return moduleName;
    }

    // ==================== SIMULATION ====================

    /** @return the drive motor's TalonFXSimState for physics simulation */
    public com.ctre.phoenix6.sim.TalonFXSimState getDriveMotorSimState() {
        return driveMotor.getSimState();
    }

    /** @return the steer motor's TalonFXSimState for physics simulation */
    public com.ctre.phoenix6.sim.TalonFXSimState getSteerMotorSimState() {
        return steerMotor.getSimState();
    }
}
