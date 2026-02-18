package frc.robot.constants;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import frc.robot.RobotConfig;

public final class SwerveConstants {
    // ==================== CAN BUS ====================
    public static final CANBus kCANivoreBus = new CANBus("CANivore");

    // ==================== GYRO ====================
    // Pigeon2 CAN ID — set in RobotConfig
    public static final int kPigeonId = RobotConfig.kPigeonId;

    // ==================== MK4i MODULE PHYSICAL CONSTANTS ====================
    // MK4i L2 gear ratios (change if using L1, L3, or L4)
    // L1 = 8.14:1, L2 = 6.75:1, L3 = 6.12:1, L4 = 5.14:1
    public static final double kDriveGearRatio = RobotConfig.kDriveGearRatio;  // L2
    public static final double kSteerGearRatio = RobotConfig.kSteerGearRatio; // ~21.43:1 for all MK4i variants

    // Wheel diameter (4 inches nominal)
    public static final double kWheelDiameterMeters      = RobotConfig.kWheelDiameterMeters;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // ==================== CHASSIS DIMENSIONS ====================
    // Distance between centers of left and right modules (track width)
    // Distance between centers of front and back modules (wheelbase)
    // Values are set in RobotConfig.java — edit there for new robot deployments.
    public static final double kTrackWidthMeters = RobotConfig.kTrackWidthMeters;
    public static final double kWheelBaseMeters  = RobotConfig.kWheelBaseMeters;

    // Distance from robot center to the furthest module
    public static final double kDriveBaseRadiusMeters =
            Math.hypot(kTrackWidthMeters / 2.0, kWheelBaseMeters / 2.0);

    // ==================== KINEMATICS ====================
    // Module positions relative to robot center (FL, FR, BL, BR)
    public static final SwerveDriveKinematics kSwerveKinematics = new SwerveDriveKinematics(
            new Translation2d(kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),   // Front Left
            new Translation2d(kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0),  // Front Right
            new Translation2d(-kWheelBaseMeters / 2.0, kTrackWidthMeters / 2.0),  // Back Left
            new Translation2d(-kWheelBaseMeters / 2.0, -kTrackWidthMeters / 2.0)  // Back Right
    );

    // ==================== SPEED LIMITS ====================
    // Derived from Kraken X60 specs: 6000 RPM = 100 RPS free speed
    private static final double kKrakenFreeSpeedRPS = 100.0;
    public static final double kTheoreticalMaxSpeedMPS =
            (kKrakenFreeSpeedRPS / kDriveGearRatio) * kWheelCircumferenceMeters;
    // 95% of theoretical to account for real-world friction and voltage drop
    public static final double kMaxSpeedMetersPerSecond = kTheoreticalMaxSpeedMPS * 0.95;
    public static final double kMaxAngularSpeedRadiansPerSecond = 2 * Math.PI; // ~1 rotation/s

    // ==================== DRIVE MOTOR PID / FF ====================
    // Values set in RobotConfig.java — edit there for new robot deployments.
    public static final double kDriveP = RobotConfig.kDriveP;
    public static final double kDriveI = RobotConfig.kDriveI;
    public static final double kDriveD = RobotConfig.kDriveD;
    public static final double kDriveS = RobotConfig.kDriveS;   // Static friction feedforward
    public static final double kDriveV = RobotConfig.kDriveV;   // Velocity feedforward (V * rps = volts)
    public static final double kDriveA = RobotConfig.kDriveA;   // Acceleration feedforward

    // ==================== STEER MOTOR PID ====================
    public static final double kSteerP = RobotConfig.kSteerP;
    public static final double kSteerI = RobotConfig.kSteerI;
    public static final double kSteerD = RobotConfig.kSteerD;

    // ==================== AUTO TRAJECTORY FOLLOWING PID ====================
    // These are used by the Choreo trajectory follower to correct position error
    public static final double kAutoTranslationP = RobotConfig.kAutoTranslationP;
    public static final double kAutoTranslationI = RobotConfig.kAutoTranslationI;
    public static final double kAutoTranslationD = RobotConfig.kAutoTranslationD;
    public static final double kAutoRotationP    = RobotConfig.kAutoRotationP;
    public static final double kAutoRotationI    = RobotConfig.kAutoRotationI;
    public static final double kAutoRotationD    = RobotConfig.kAutoRotationD;

    // ==================== CURRENT LIMITS ====================
    // Values set in RobotConfig.java — edit there for new robot deployments.
    public static final double kDriveCurrentLimit        = RobotConfig.kDriveCurrentLimit;
    public static final double kDriveStatorCurrentLimit  = RobotConfig.kDriveStatorCurrentLimit;
    public static final double kSteerCurrentLimit        = RobotConfig.kSteerCurrentLimit;

    // ==================== SLEW RATE LIMITING (smoother driver control) ====================
    public static final double kTranslationSlewRate = RobotConfig.kTranslationSlewRate; // m/s per second
    public static final double kRotationSlewRate    = RobotConfig.kRotationSlewRate;    // rad/s per second

    // ==================== HEADING LOCK PID ====================
    // Holds current heading when driver releases rotation stick.
    // ProfiledPIDController with trapezoidal velocity constraints.
    public static final double kHeadingLockP = RobotConfig.kHeadingLockP;
    public static final double kHeadingLockI = RobotConfig.kHeadingLockI;
    public static final double kHeadingLockD = RobotConfig.kHeadingLockD;
    public static final double kMaxAngularAccelRadiansPerSecondSq = RobotConfig.kMaxAngularAccelRadiansPerSecondSq;

    // ==================== SLOW MODE ====================
    // Multipliers applied when slow mode trigger is held.
    // Fine positioning near scoring positions or during defense.
    public static final double kSlowModeSpeedMultiplier = RobotConfig.kSlowModeSpeedMultiplier;
    public static final double kSlowModeRotMultiplier   = RobotConfig.kSlowModeRotMultiplier;

    // ==================== ODOMETRY ====================
    public static final double kOdometryPeriodSeconds = 0.004;    // 250Hz
    public static final double kMainLoopPeriodSeconds = 0.02;     // 50Hz

    // ==================== SIGNAL UPDATE FREQUENCIES ====================
    public static final double kPigeonYawUpdateFreqHz       = 100.0;
    public static final double kPigeonAngVelUpdateFreqHz    = 100.0;
    public static final double kModulePositionUpdateFreqHz  = 100.0;
    public static final double kModuleVelocityUpdateFreqHz  = 50.0;

    // ==================== VOLTAGE LIMITS ====================
    public static final double kPeakForwardVoltage  = 12.0;
    public static final double kPeakReverseVoltage  = -12.0;

    // ==================== SLIP DETECTION ====================
    public static final double kSlipDetectionThreshold = 0.30;    // 30% discrepancy
    public static final double kSlipMinCommandedSpeed  = 0.5;     // m/s
    public static final double kSlipMinActualSpeed     = 0.1;     // m/s

    // ==================== MODULE DRIVE THRESHOLDS ====================
    public static final double kAntiJitterSpeedMPS = 0.05;       // hold last angle below this

    // ==================== HEADING LOCK TOLERANCE ====================
    public static final double kHeadingLockToleranceDeg = 1.5;

    // ==================== VISION ====================
    public static final double kMaxVisionMeasurementAgeSec = 0.100; // reject stale measurements

    // ==================== MODULE HEALTH ====================
    public static final double kMaxSignalAgeSeconds = 0.5;       // stale CAN signal threshold

    // ==================== SIMULATION ====================
    public static final double kDriveSimMOI = 0.025;             // kg*m^2 (wheel + gearbox)
    public static final double kSteerSimMOI = 0.004;             // kg*m^2 (module azimuth)

    // ==================== MODULE CAN IDs AND OFFSETS ====================
    // Front Left Module
    public static final int kFrontLeftDriveMotorId   = RobotConfig.kFrontLeftDriveMotorId;
    public static final int kFrontLeftSteerMotorId   = RobotConfig.kFrontLeftSteerMotorId;
    public static final int kFrontLeftEncoderId      = RobotConfig.kFrontLeftEncoderId;
    public static final double kFrontLeftEncoderOffset = RobotConfig.kFrontLeftEncoderOffset;

    // Front Right Module
    public static final int kFrontRightDriveMotorId   = RobotConfig.kFrontRightDriveMotorId;
    public static final int kFrontRightSteerMotorId   = RobotConfig.kFrontRightSteerMotorId;
    public static final int kFrontRightEncoderId      = RobotConfig.kFrontRightEncoderId;
    public static final double kFrontRightEncoderOffset = RobotConfig.kFrontRightEncoderOffset;

    // Back Left Module
    public static final int kBackLeftDriveMotorId   = RobotConfig.kBackLeftDriveMotorId;
    public static final int kBackLeftSteerMotorId   = RobotConfig.kBackLeftSteerMotorId;
    public static final int kBackLeftEncoderId      = RobotConfig.kBackLeftEncoderId;
    public static final double kBackLeftEncoderOffset = RobotConfig.kBackLeftEncoderOffset;

    // Back Right Module
    public static final int kBackRightDriveMotorId   = RobotConfig.kBackRightDriveMotorId;
    public static final int kBackRightSteerMotorId   = RobotConfig.kBackRightSteerMotorId;
    public static final int kBackRightEncoderId      = RobotConfig.kBackRightEncoderId;
    public static final double kBackRightEncoderOffset = RobotConfig.kBackRightEncoderOffset;
}
